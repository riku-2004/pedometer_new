WINDOW_CENTER = 7
SENSITIVITY = 1700
TIME_1_0_SECOND = 50
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
  REG_DATA = 0x0E

  def initialize(i2c)
    @i2c = i2c
  end

  def conv(ary, base)
    ((ary.getbyte(base) << 24) | (ary.getbyte(base+1) << 16)) >> 18
  end

  def read()
    @i2c.write(DEV_ADDR, [REG_DATA])
    data = @i2c.read(DEV_ADDR, 6)
    x = conv(data, 0)
    y = conv(data, 2)
    z = conv(data, 4)
    ADXLResult.new(x, y, z)
  end
end

class Pedometer
  def initialize()
    @window = Array.new(15, 0)
    @window_filled = false
    @current_state = 0
    @time_since_mountain = 0
    @max_value = 0
    @consecutivesteps = 0
    @step_count = 0
    @fill_count = 0
  end
  def step_algorithm_an2554(mag)
    # i = @window.length - 1 のとき @window[i + 1] が範囲外になるので、0..(@window.length - 2) までの範囲でループする
    for i in 0..(@window.length - 2)
      @window[i] = @window[i + 1]
    end
    @window[@window.length - 1] = mag
    if !@window_filled
      @fill_count+=1
      if @fill_count >= @window.length
        @window_filled = true
        return
      end
    end
    @center_val = @window[WINDOW_CENTER]
    @is_max = true
    @is_min = true
    for i in 0..(@window.length - 1)
      if i == WINDOW_CENTER
        next
      end
      if @window[i] >= @center_val 
        @is_max = false
      end
      if @window[i] <= @center_val
        @is_min = false
      end
    end
    if @current_state == 1
      @time_since_mountain+=1
    end

    case @current_state
    when 0
      if @is_max
        @max_value = @center_val
        @current_state = 1
        @time_since_mountain = 0
      end
    when 1
      if @is_min
        if @max_value - @center_val > SENSITIVITY
          @consecutivesteps+=1
          if @consecutivesteps == 4
            @step_count+=4
            puts("Step detected! Total steps: #{@step_count}")
          elsif @consecutivesteps > 4
            @step_count+=1
            puts("Step detected! Total steps: #{@step_count}")
          end
        else 
          @consecutivesteps = 0
          puts("False step detected. Total steps: #{@step_count}")
        end
        @current_state = 0
      elsif @time_since_mountain > TIME_1_0_SECOND
        @consecutivesteps = 0
        @current_state = 0
      end
    end
  end
end

# センサーとアルゴリズムの初期化                                                                                                                                                                                                           
i2c = I2C.new()                                                                                                                                                                                                                            
adxl = ADXL367.new(i2c)
pedometer = Pedometer.new()

# センサーを計測モードにする（0x2D レジスタに 0x02 を書く）
i2c.write(0x1D, [0x2D, 0x02])

# フィルタリング用バッファ
BUFFER_SIZE = 4
circ_buffer = Array.new(BUFFER_SIZE, 0)
buf_index = 0

# メインループ
while true
  result = adxl.read()
  raw_mag = Math.sqrt(result.x * result.x + result.y * result.y + result.z * result.z).to_i
  circ_buffer[buf_index] = raw_mag
  buf_index = (buf_index + 1) % BUFFER_SIZE
  sum = 0
  for i in 0..(BUFFER_SIZE - 1)
    sum += circ_buffer[i]
  end
  filtered_mag = sum / BUFFER_SIZE
  pedometer.step_algorithm_an2554(filtered_mag)
  Copro.delayMs(20)
end

