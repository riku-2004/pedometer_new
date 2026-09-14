WINDOW_CENTER = 8
SENSITIVITY = 1700
ONE_SECOND = 50
FILTER_ORDER = 4
THRESHOLD_ORDER = 4
INIT_OFFSET_VALUE = 4000
class I2C
    def initialize()
      Copro.i2cinit()
    end
    def read(addr, len)
      Copro.i2cread(addr, len)
    end
    def write(addr, data)
      Copro.i2cwrite(addr, data)
    end
  end
  class ADXLResult
    attr_accessor :x, :y, :z
    def initialize(x, y, z)
      @x = x; @y = y; @z = z
    end
  end
  class ADXL367
    DEV_ADDR = 0x1D
    CMD_MEASURE = [0x2D, 2]
    CMD_DATA = [0x0E]
    def initialize(i2c)
      @i2c = i2c
    end

    def on()
      @i2c.write(DEV_ADDR, CMD_MEASURE)
    end

    def conv(ary, base)
      ((ary.getbyte(base) << 24) | (ary.getbyte(base+1) << 16)) >> 18
    end
    def read()
      @i2c.write(DEV_ADDR, CMD_DATA)
      Copro.delayMs(5)
      val = @i2c.read(DEV_ADDR, 6)
      if val.size == 0 then
        return nil
      end
      ADXLResult.new(conv(val, 0), conv(val, 2), conv(val, 4))
    end
  end

  class Pedometer
    # center_val と step_count を外部から読み取れるようにする（SPIFFSへのログ記録用）
    attr_reader :center_val, :step_count
    def initialize()
      # 17個の要素を持つ配列を作成し、すべての要素を0で初期化する
      @window = Array.new(17, 0)
      @window_filled = false
      @current_state = 0
      @time_since_mountain = 0
      @max_value = 0
      @consecutivesteps = 0
      @step_count = 0
      @fill_count = 0
      @filter_mean_buffer = 0
      @index_average = 0
      @index_threshold = 0
      @threshold_sum = THRESHOLD_ORDER * INIT_OFFSET_VALUE
      @buffer_dynamic_threshold = Array.new(THRESHOLD_ORDER, INIT_OFFSET_VALUE)
      @buffer_raw = Array.new(FILTER_ORDER, 0)
      @oldThreshold = INIT_OFFSET_VALUE
      @flag_threshold_counter = 0
    end
    def step(x, y, z)
      mag = x.abs + y.abs + z.abs
      #FilterMeanBuffer:バッファの合計値
      @filter_mean_buffer = @filter_mean_buffer - @buffer_raw[@index_average] + mag #平均から最後の値を引き、新しい値を足す
      @filter_module_data = @filter_mean_buffer / FILTER_ORDER #平均を計算
      @buffer_raw[@index_average] = mag #フィルタリングされていないバッファにモジュールを格納
      # i = @window.length - 1 のとき @window[i + 1] が範囲外になるので、0..(@window.length - 2) までの範囲でループする
      for i in 0..(@window.length - 2)
        @window[i] = @window[i + 1]
      end
      @window[@window.length - 1] = @filter_module_data
      if !@window_filled
        @fill_count += 1
        if @fill_count >= @window.length
          @window_filled = true
          return
        end
      end
      @center_val = @window[WINDOW_CENTER]
      is_max = true
      is_min = true
      for i in 0..(@window.length - 1)
        if i == WINDOW_CENTER
          next
        end
        if @window[i] >= @center_val
          is_max = false
        end
        if @window[i] <= @center_val
          is_min = false
        end
      end
      if @current_state == 1
        @time_since_mountain += 1
      end

      case @current_state
      when 0
        if is_max
          @max_value = @center_val
          @current_state = 1
          @time_since_mountain = 0
        end
      when 1
        if is_min
          diff = @max_value - @center_val
          if diff > SENSITIVITY
            newThreshold = (@max_value + @center_val) / 2
            @threshold_sum = @threshold_sum - @buffer_dynamic_threshold[@index_threshold] + newThreshold
            @oldThreshold = @threshold_sum / THRESHOLD_ORDER
            @buffer_dynamic_threshold[@index_threshold] = newThreshold
            @index_threshold += 1
            if @index_threshold > THRESHOLD_ORDER - 1
              @index_threshold = 0
            end
          end
          if @max_value > @oldThreshold + SENSITIVITY / 2 && @center_val < @oldThreshold - SENSITIVITY / 2
            @flag_threshold_counter = 0
            @consecutivesteps += 1
            if @consecutivesteps == 4
              @step_count += 4
              # putsは消去
            elsif @consecutivesteps > 4
              @step_count += 1
            end
          else
            @flag_threshold_counter += 1
            if @flag_threshold_counter > 1
              @flag_threshold_counter = 0
              @consecutivesteps = 0
            end
          end
          @current_state = 0
        elsif @time_since_mountain > ONE_SECOND
          @consecutivesteps = 0
          @current_state = 0
        end
      end
      @index_average += 1
      if @index_average > FILTER_ORDER - 1
        @index_average = 0
      end
    end
  end

  # 初期化
  i2c = I2C.new()
  acc = ADXL367.new(i2c)
  acc.on()
  pedometer = Pedometer.new()
  spiffs = Spiffs.new
  spiffs.init
  time_ms = 0
  ARRAY_SIZE = 70
  buf_x = Array.new(ARRAY_SIZE, 0)
  buf_y = Array.new(ARRAY_SIZE, 0)
  buf_z = Array.new(ARRAY_SIZE, 0)

  # メインループ
  while true
    puts("before sleep")
    # LP Core: データ収集のみ
    Copro.sleep_and_run do
      count = 0
      while count < ARRAY_SIZE
        result = acc.read()
        buf_x[count] = result.x
        buf_y[count] = result.y
        buf_z[count] = result.z
        count += 1
        Copro.delayMs(20)
      end
    end
    # HP Core: 歩数計算 + SPIFFS記録
    count = 0
    while count < ARRAY_SIZE
      pedometer.step(buf_x[count], buf_y[count], buf_z[count])
      count += 1
    end
    time_ms += ARRAY_SIZE * 20
    spiffs.write("#{time_ms},#{pedometer.center_val},#{pedometer.step_count}")
    puts("after sleep")
  end