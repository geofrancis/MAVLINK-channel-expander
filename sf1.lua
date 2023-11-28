local function run()

    local globalVarOffset = 0
    local globalVarBinary = 0
    local globalVarOutput = 0

    -- Get Info about Channel 8
    --mixTable = model.getMix(7, 0)

    for channel = 17, 32 do
        
        -- Zero out the globalVarOffset at the beginning of the cycle
        if channel == 17 then
            globalVarOffset = 0
        end

        -- Read the value of the channel, returns -1024 to 1024
        local channelValue = getValue("ch" .. channel)
        
        -- Interpret <= 0 as 0 and > 0 as 1
        local globalVarBinary = (channelValue > 0) and 1 or 0  

        -- Generate the output we want
        globalVarOutput = globalVarOffset + globalVarBinary

        local startTime = getTime()
        local delayTime = 40  -- 400 milliseconds

        while getTime() - startTime < delayTime do
        end

        model.setGlobalVariable(0, 0, globalVarOutput)
        
        -- Prepare variables for next channel
        globalVarOffset = globalVarOffset + 5
        globalVarBinary = 0
        globalVarOutput = 0

    end

    return 0
end

return { run=run }