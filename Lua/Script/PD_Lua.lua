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

    Kp = {}
    Kv = {}
    
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
    tt = 1
    step = 'end'

    -- Get handles for the UR5 joints
    ur5Joints = {}
    for i = 1, 6 do
        ur5Joints[i] = sim.getObject("/Joint" .. i)
    end

    _done = 0
    _genScurve = 1
    save = {}
    
    print("Start simulation")
end

function sysCall_actuation()
    if step == 'ini' then
        for i = 1, 6 do 
            Pini[i] = sim.getJointPosition(ur5Joints[i])
        end
        _genScurve, Cmd = UR.genScurve(Pini, Pend, params)
        if _genScurve then
            step = 'run'
            print("Running PTP ...")
        else
            step = 'stay'
        end
        -- _done = _done + 1
    elseif step == 'run' then 
        PTP()
    elseif step == 'stay' then
        Stay()
    else
        -- print(step)
    end
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
            -- PD-like Controller
            Record.TorCtrl[tt][i] = Kv[i] * (Kp[i] * (Cmd.P[tt][i] - Record.P[tt][i]) - Record.V[tt][i])
            -- PD Controller
            -- Record.TorCtrl[t][i] = Kp[i] * (Cmd.P[t][i] - Record.P[t][i])-- + Kv[i] * (Cmd.V[t][i] - Record.V[t][i])
            sim.setJointTargetForce(ur5Joints[i], Record.TorCtrl[tt][i])
        end
    else
        step = 'stay'
        local filePath = sim.getStringParameter(sim.stringparam_scene_path) .. "/result/record.txt"
        if save[1] == 1 then
            UR.SaveData(Cmd, Record, filePath)
        end
        return
    end
    tt = tt + 1
end

function Stay()
    sim.pauseSimulation()
    -- step = 'wait'
    Pend = {}
    Kp = {}
    Kv = {}
end 

function set_Env_Py(inInts, inFloats, inStrings, inBuffer)
    local outInts = {}
    local outFloats = {}
    local outStrings = {}
    local outBuffer = ''
    for i = 1, 6 do
        -- table.insert(Pini, inFloats[i])
        table.insert(Pend, inFloats[i])
        table.insert(Kp, inFloats[i+6])
        table.insert(Kv, inFloats[i+12])
    end

    print('Kp', Kp)
    print('Kv', Kv)
    step = 'ini'
    tt = 1
    Record = {
        P = {},         
        V = {},        
        TorCtrl = {}
    }
    table.insert(save, inInts[1])
    print(save)
    return outInts, outFloats, outStrings, outBuffer
end

function return_Error_Py(inInts, inFloats, inStrings, inBuffer)
    
    local outInts = {}
    local outFloats = {}
    local outStrings = {}
    local outBuffer = ''

    table.insert(outInts, _genScurve)
    table.insert(outInts, _done)

    avgError = UR.PosError(Cmd, Record, "joint_all") / #Cmd.P
    table.insert(outFloats, avgError)

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