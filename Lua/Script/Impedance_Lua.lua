function sysCall_init()
    mat = require("matrix")
    UR  = require("robot")
    pi  = math.pi
    params = {
        Amax = {   4,    4,    4,    4,    4,    4}, -- Max Angular Acceleration for each axis
        aavg = { 0.9,  0.9,  0.9,  0.9,  0.9,  0.9}, -- Average Angular Acceleration for each axis
        Vmax = {   1,  0.5,    1,  0.5,    1,    1}, -- Max Angular Velocity for each axis
        Axis = 6,                                    -- Number of Axis
        sampT = 0.001,                               -- Sampling Time
    }

    Pini = {} -- Initial Position for each axis
    Pend = {} -- Goal Position for each axis

    Kp = {738.05, 2025.92, 1402.39, 1710.42, 323.64, 1199.65}
    Kv = {  0.75,    1.88,    0.92,    1.44,   2.12,    1.96}

    -- Joint space
    md = 50 * mat:new(6, "I")
    kd = 500 * mat:new(6, "I")
    bd = 1000 * mat:new(6, "I")
    
    mdinv    = mat.invert(md)
    mdinv_bd = mat.invert(md):mul(bd)
    mdinv_kd = mat.invert(md):mul(kd)
    
    E   = mat.transpose({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
    dE  = mat.transpose({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
    ddE = mat.transpose({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}})
    force_Desired = mat{0.0, 0.0, -10.0}
    
    Cmd  = {
        P = {},         -- Position from getJointPosition
        V = {},         -- Velocity from getJointVelocity
        TorCtrl = {},   -- Torque from controller
    }
    Record = {
        P = {},         -- Position from getJointPosition
        V = {},         -- Velocity from getJointVelocity
        TorCtrl = {},   -- Torque from controller
    }

    tt_ini = 1
    tt = 1
    step = 'ini'

    -- Get handles for the UR5 joints
    ur5Joints = {}
    for i = 1, 6 do
        ur5Joints[i] = sim.getObject("/Joint" .. i)
    end
    force_sensor = sim.getObject('/connection')

    _done = 1
    _genScurve = 1
    save = {}
    cmd_path = sim.getStringParameter(sim.stringparam_scene_path):sub(1, #sim.getStringParameter(sim.stringparam_scene_path):match(".*[/\\]") - 1) .. "/data/input/command.txt"
    Pd = {}
    fext = {}
    print("Start simulation")
end

function sysCall_actuation()
    if step == 'ini' then
        for i = 1, 6 do 
            Pini[i] = sim.getJointPosition(ur5Joints[i])
        end

        Cmd = read_Cmd(cmd_path, 3)
        Pend =  Cmd.P[1]
        _genScurve, Cmd_ini = UR.genScurve(Pini, Pend, params)

        if _genScurve then
            step = 'run2ini'
            print("Running PTP ...")
        else
            step = 'stay'
        end
    elseif step == 'run2ini' then 
        PTP_ini()
    elseif step == 'impedance' then
        -- PTP()
        PTP_Admittance()
    elseif step == 'stay' then
        Stay()
    else
        -- print(step)
    end
end

function PTP_ini()
    if tt_ini <= #Cmd_ini.P then
        --Record.P[tt_ini] = {}
        --Record.V[tt_ini] = {}
        Record.TorCtrl[tt_ini] = {}
        for i = 1, 6 do
            -- Get Joint position
            P = sim.getJointPosition(ur5Joints[i])
            V = sim.getJointVelocity(ur5Joints[i])
            -- PD-like Controller
            Record.TorCtrl[tt_ini][i] = Kv[i] * (Kp[i] * (Cmd_ini.P[tt_ini][i] - P) - V)
            sim.setJointTargetForce(ur5Joints[i], Record.TorCtrl[tt_ini][i])
        end
    else
        step = 'impedance'
    end
    tt_ini = tt_ini + 1
end

function PTP()
    if tt <= #Cmd.P then
        Record.P[tt] = {}
        Record.V[tt] = {}
        Record.TorCtrl[tt] = {}
        for i = 1, 6 do
            -- Get Joint position
            Record.P[tt][i] = sim.getJointPosition(ur5Joints[i])
            Record.V[tt][i] = sim.getJointVelocity(ur5Joints[i])
            -- Admittnace Controller

            -- PD-like Controller
            Record.TorCtrl[tt][i] = Kv[i] * (Kp[i] * (Cmd.P[tt][i] - Record.P[tt][i]) - Record.V[tt][i])
            sim.setJointTargetForce(ur5Joints[i], Record.TorCtrl[tt][i])
        end
    else
        step = 'stay'
        local filePath = sim.getStringParameter(sim.stringparam_scene_path):sub(1, #sim.getStringParameter(sim.stringparam_scene_path):match(".*[/\\]") - 1) .. "/data/result/record.txt"
        UR.saveData(Cmd, Record, filePath)
        return
    end
    tt = tt + 1
end

function PTP_Admittance()
    if tt <= #Cmd.P then
        Record.P[tt] = {}
        Record.V[tt] = {}
        Record.TorCtrl[tt] = {}
        local P_error = {}
        local V_error = {}
        Pd[tt] = {}
        
        for i = 1, 6 do
           Record.P[tt][i] = sim.getJointPosition(ur5Joints[i])
           Record.V[tt][i] = sim.getJointVelocity(ur5Joints[i])
           P_error[i] = Cmd.P[tt][i] - Record.P[tt][i]
           V_error[i] = Cmd.V[tt][i] - Record.V[tt][i]
        end
        
        result, forceVector, torqueVector = sim.readForceSensor(force_sensor)
            
        force = mat{0.0, 0.0, forceVector[3]}
        fext[tt] = forceVector[3]
        JacoCmd_ = UR.Jacobian(Record.P[tt])
        invJacoCmd_ = mat.transpose(JacoCmd_):mul(mat.invert(JacoCmd_:mul(mat.transpose(JacoCmd_))))
        -- print("force", force)
        -- print("force_Des", force_Desired)

        tau_error = invJacoCmd_:mul(force-force_Desired)

        -- Joint space
        ddE = mat.add(mdinv_bd:mul(mat.transpose({V_error})), mdinv_kd:mul(mat.transpose({P_error})))
        ddE = mat.add(ddE, mdinv:mul(tau_error))
        dE  = mat.add( dE, ddE * params.sampT)
        E   = mat.add(  E,  dE * params.sampT)
        Pd[tt]  = mat.add(mat.transpose({Cmd.P[tt]}), E)
        

        for i = 1, 6 do
            if i==3 then
                Record.TorCtrl[tt][i] = Kv[i] * (Kp[i] * (Pd[tt][i][1] - Record.P[tt][i]) - Record.V[tt][i])
            else
                Record.TorCtrl[tt][i] = Kv[i] * (Kp[i] * (Cmd.P[tt][i] - Record.P[tt][i]) - Record.V[tt][i])
            end
            sim.setJointTargetForce(ur5Joints[i], Record.TorCtrl[tt][i])
        end
    else
        step = 'stay'
    end
    tt = tt + 1
end

function Stay()
    sim.pauseSimulation()
    -- step = 'wait'
    Pend = {}
end 

function set_Env_Py(inInts, inFloats, inStrings, inBuffer)
    local outInts = {}
    local outFloats = {}
    local outStrings = {}
    local outBuffer = ''
    md_temp = inFloats[1]
    bd_temp = inFloats[2]
    kd_temp = inFloats[3]
    
    md = md_temp * mat:new(6, "I")
    kd = bd_temp * mat:new(6, "I")
    bd = kd_temp * mat:new(6, "I")
    mdinv    = mat.invert(md)
    mdinv_bd = mat.invert(md):mul(bd)
    mdinv_kd = mat.invert(md):mul(kd)
    
    print("md : ", md_temp, "bd : ", bd_temp, "kd : ", kd_temp)

    step = 'ini'
    tt = 1
    Record = {
        P = {},         
        V = {},        
        TorCtrl = {}
    }
    table.insert(save, inInts[1])
    return outInts, outFloats, outStrings, outBuffer
end

function return_Error_Py(inInts, inFloats, inStrings, inBuffer)
    
    local outInts = {}
    local outFloats = {}
    local outStrings = {}
    local outBuffer = ''

    table.insert(outInts, _genScurve)
    table.insert(outInts, _done)
    
    error = 0
    for i = 1, #fext do 
        error = error + math.abs((-10 - fext[i]))
    end
    error = error/#fext
    table.insert(outFloats, error)
    
    effError = UR.PosError(Cmd, Record, "eff_end") * 1000
    table.insert(outFloats, effError)

    return outInts, outFloats, outStrings, outBuffer
end

function get_State_Py(inInts, inFloats, inStrings, inBuffer)
    print("get state ...")
    local outInts = {}
    local outFloats = {}
    local outStrings = {}
    local outBuffer = ''
    for i = 1, 6 do
        table.insert(outFloats, sim.getJointPosition(ur5Joints[i]))
    end
    return outInts, outFloats, outStrings, outBuffer
end

function sysCall_cleanup()
    -- do some clean-up here
end

function read_Cmd(filePath, sol)
    -- Specify the path to the .txt file you want to load
    -- local filePath = sim.getStringParameter(sim.stringparam_scene_path) .. filePath

    -- Open the file for reading
    local file = io.open(filePath, 'r')
    if file then
        -- Initialize empty tables to store position, velocity, and acceleration commands
        local read_Cmd  = {
            P = {},
            V = {},
            A = {},
        }
        
        local line_idx = 1
        
        for line in file:lines() do
            read_Cmd.P[line_idx] = {}
            read_Cmd.V[line_idx] = {}
            read_Cmd.A[line_idx] = {}
            local values = {}
            local value_idx = 1
            for value in line:gmatch("%S+") do
                values[value_idx] = tonumber(value)
                value_idx = value_idx + 1
            end
            if #values == 18*8 then
                for j = 1, 6 do
                    read_Cmd.P[line_idx][j] = values[j+(sol-1)*18]
                    read_Cmd.V[line_idx][j] = values[j+6+(sol-1)*18]
                    read_Cmd.A[line_idx][j] = values[j+12+(sol-1)*18]
                end
                line_idx = line_idx + 1
            else
                print("Invalid line: " .. line)
            end
        end
        -- Close the file
        file:close()
        return read_Cmd
    else
        print("Failed to open the file: " .. filePath)
        return 0
    end
end