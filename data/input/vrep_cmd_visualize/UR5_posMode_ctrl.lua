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

    Pini = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} -- Initial Position for each axis
    Pend = {} -- Goal Position for each axis

    Kp = {}
    Kv = {}
    
    Cmd  = {
        P = {},         -- Position from getJointPosition
        V = {},         -- Velocity from getJointVelocity
        TorCtrl = {},   -- Torque from controller
    }

    -- Get handles for the UR5 joints
    ur5Joints = {}
    for i = 1, 6 do
        ur5Joints[i] = sim.getObject("/Joint" .. i)
    end
    
    print("Start simulation")
    -- file = "/command.txt"
    file = "D:/Group/NSTC2024/Scene/CoppeliaSim/UR5_SAC_parameters_tuning/data/input/command.txt"
    Cmd = read_Cmd(file, 3)
    tt = 1
    
    Pini = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0} -- Initial Position for each axis
    Pend =  Cmd.P[1]-- Goal Position for each axis
    
    rec, Cmd_ini = UR.genScurve(Pini, Pend, params)
    j = 1

    All_Cmd  = {
        P = {},
        V = {}
    }
    All_Record  = {
        P = {},
        V = {}
    }

end

function sysCall_actuation()
    if j < #Cmd_ini.P then
        for i = 1, 6 do
            sim.setJointTargetPosition(ur5Joints[i], Cmd_ini.P[j][i])
            All_Cmd.P[tt+j-1][i] = Cmd_ini.P[j][i]
            All_Cmd.V[tt+j-1][i] = Cmd_ini.V[j][i]
            All_Record.P[tt+j-1][i] = sim.getJointPosition(ur5Joints[i])
            All_Record.V[tt+j-1][i] = sim.getJointVelocity(ur5Joints[i])
        end
        j = j + 1
    elseif tt < #Cmd.P then
        for i = 1, 6 do
            sim.setJointTargetPosition(ur5Joints[i], Cmd.P[tt][i])
            All_Cmd.P[tt+j-1][i] = Cmd.P[j][i]
            All_Cmd.V[tt+j-1][i] = Cmd.V[j][i]
            All_Record.P[tt+j-1][i] = sim.getJointPosition(ur5Joints[i])
            All_Record.V[tt+j-1][i] = sim.getJointVelocity(ur5Joints[i])
        end
        tt = tt + 1
        print("eff : ", UR.FK(Cmd.P[tt]))
    else
        local path = 'record.txt'
        UR.SaveData(All_Cmd, All_Record, path)
        sim.stopSimulation()
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
            sim.setJointTargetForce(ur5Joints[i], Record.TorCtrl[tt][i])
        end
    else
        step = 'stay'
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
