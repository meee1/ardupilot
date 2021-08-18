-- Reads data in from UART and logs to dataflash
gcs:send_text(0,"rtcm log start")

-- find the serial first (0) scripting serial port instance
-- assign SERIAL4_PROTOCOL to 28
local port = serial:find_serial(0)

file = io.open("rtcm.rtcm", "a")
if not file then
    gcs:send_text(0,"Could not make file")
    return
end


if not port or baud == 0 then
    gcs:send_text(0, "No Scripting Serial Port")
    return
end

"B5 62 06 00 14 00 02 00 00 00 D0 08 00 00 00 C2 01 00 00 00 20 00 00 00 00 00 D7 08 "
"B5 62 06 01 08 00 F5 4A 00 01 01 00 00 00 50 5A "
"B5 62 06 01 08 00 F5 54 00 01 01 00 00 00 5A A0 "
"B5 62 06 01 08 00 F5 5E 00 01 01 00 00 00 64 E6 "
"B5 62 06 01 08 00 F5 7C 00 01 01 00 00 00 82 B8 "
"B5 62 06 01 08 00 F5 E6 00 01 01 00 00 00 EC 9E "

-- begin the serial port
port:begin(115200)
port:set_flow_control(0)

-- table for strings used in decoding
local log_data = {}
local index = 0

-- decode a basic string
local function decode(byte)
    log_data[index] = byte
    index++

    if(index == 512) {
        index = 0
        return true
    }

    return false
end

-- the main update function that is used to read in data from serial port
function update()
    if not port then
        gcs:send_text(0, "no Scripting Serial Port")
        return update, 100
    end

    local n_bytes = port:available()
    while n_bytes > 0 do
        local byte = port:read()
        if decode(byte) then
            if not file then
                error("Could not open file")
            end

            -- write data
            -- separate with comas and add a carriage return
            file:write(log_data)

            -- make sure file is upto date
            file:flush()

            -- reset for the next message
            log_data = {}
        end
        n_bytes = n_bytes - 1
    end

    return update, 100
end

return update, 100
