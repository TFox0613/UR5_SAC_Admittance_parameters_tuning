local robot = {_TYPE='module', _NAME='robot', _VERSION='0.1'}
local mat   = require("matrix")
local pi = math.pi


-- local fucntion definition
local function sign(x)
    if x >= 0 then
        return 1
    else
        return 0
    end
end

local function checkNil(x)
    if x ~= nil then
        return x
    else
        return " "
    end 
end

local function cos(val)
    return math.cos(val)
end

local function sin(val)
    return math.sin(val)
end

local function Tij(j, theta)
    local d    = { 0.089159,      0,        0,   0.10915,   0.09465,  0.0823}
    local a    = {        0, -0.425, -0.39225,         0,         0,       0}
    local alph = {     pi/2,      0,        0,      pi/2,     -pi/2,       0}
    
    local T_a = mat{{1, 0, 0, a[j]},
                    {0, 1, 0,    0},
                    {0, 0, 1,    0},
                    {0, 0, 0,    1}}

    local T_d = mat{{1, 0, 0,    0},
                    {0, 1, 0,    0},
                    {0, 0, 1, d[j]},
                    {0, 0, 0,    1}}

    local Rzt = mat{{cos(theta[j]), -sin(theta[j]), 0, 0},
                    {sin(theta[j]),  cos(theta[j]), 0, 0},
                    {            0,              0, 1, 0},
                    {            0,              0, 0, 1}}
 
    local Rxa = mat{{1,            0,             0, 0},
                    {0, cos(alph[j]), -sin(alph[j]), 0},
                    {0, sin(alph[j]),  cos(alph[j]), 0},
                    {0,            0,             0, 1}}

    local A_i = T_d:mul(Rzt):mul(T_a):mul(Rxa)
    return A_i
end


-- library callable function
function robot.genScurve(Pini, Pend, params)
    -- 7 stage Scurve
    -- ============== Input ==============
    -- Pini : (1, Axis) Initial Position
    -- Pend : (1, Axis) Goal Position
    -- params
    --   Amax : (1, Axis) Max Angular Acceleration
    --   Aavg : (1, Axis) Average Angular Acceleration ((0.5 ~ 1)*Amax)
    --   Vmax : (1, Axis) Max Angular Velocity
    --   Axis : Number of Axis

    -- ============= Output ==============
    -- Cmd
    --   P : (tf/sampT, Axis) Position Command
    --   V : (tf/sampT, Axis) Velocity Command
    --   A : (tf/sampT, Axis) Acceration Command
    --   (option) J : (tf/sampT, Axis) Jerk Command

    print("Command Generating ...")
    print("Pini : ", Pini)
    print("Pend : ", Pend)

    -- Parameters
    local Amax  = params.Amax
    local aavg  = params.aavg
    local Aavg  = {}
    local Vmax  = params.Vmax
    local Axis  = params.Axis
    local sampT = params.sampT

    local Ta = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    local Tb = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    local Tc = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    local Ts = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    local S  = mat{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}

    local ret = 0  -- check if successful (1)
    local Cmd = {
        P = {},
        V = {}, 
        A = {},
        J = {}
    }
    for i = 1, Axis do 
        S[i] = math.abs(Pend[i] - Pini[i])
        Aavg[i] = aavg[i] * Amax[i]
        if (S[i] < Vmax[i]*Vmax[i] / Aavg[i]) and (S[i] ~= 0.0) then
            Vmax[i] = math.sqrt(S[i]*Aavg[i])
        end
        if S[i] ~= 0.0 then
            Ta[i]   = Vmax[i] / Aavg[i]
            Tb[i]   = (2 * Vmax[i] / Amax[i]) - Ta[i]
            Tc[i]   = (Ta[i] - Tb[i]) / 2
            Ts[i]   = (S[i] - Vmax[i] * Ta[i]) / Vmax[i]
        end
    end

    -- Find the shortest time
    local t_idx = 1
    for i = 2, #Ts do
        if (2 * Ta[i] + Ts[i]) > (2 * Ta[t_idx] + Ts[t_idx]) then
            t_idx = i
        end
    end

    -- Time node
    local t1 = Tc[t_idx]
    local t2 = Tc[t_idx] + Tb[t_idx]
    local t3 = Ta[t_idx]
    local t4 = Ta[t_idx] + Ts[t_idx]
    local t5 = Ta[t_idx] + Ts[t_idx] + Tc[t_idx]
    local t6 = Ta[t_idx] + Ts[t_idx] + Tc[t_idx] + Tb[t_idx]
    local t7 = Ta[t_idx] + Ts[t_idx] + Ta[t_idx]

    -- check Axis to determine Jerk of each joint
    local Jerk = mat{}
    if Axis == 1 then
        Jerk = Amax * sign((Pend - Pini)) / Tc[t_idx]
    else
        for i = 1, Axis do
            Jerk[i] = (Pend[i] - Pini[i]) / (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 + 1/6 * t4^3 - 1/6 * t5^3 - 1/6 * t6^3 - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                    + t7 * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + 1/2 * t6^2 + t1 * t3 + t2 * t3)
                    + 1/2 * t7^2 * (t4 - t5 - t6) + 1/6 * t7^3)
        end
    end

    -- Generate Scurve
    local t = 0
    local ACmd_i, VCmd_i, PCmd_i = {}, {}, {}
    for i = 1, math.floor(t7 / sampT) do
        ACmd_i[i], VCmd_i[i], PCmd_i[i] = {}, {}, {}

        if t < t1 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * t
                VCmd_i[i][axis] = Jerk[axis] * (1/2 * t^2)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t^3)
            end
        elseif t1 <= t and t < t2 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * t1
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 + t1 * t)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 - 1/2 * t1^2 * t + 1/2 * t1 * t^2)
            end
        elseif t2 <= t and t < t3 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * (-t + t1 + t2)
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 + t1 * t + t2 * t - 1/2 * t^2)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + (-1/2 * t1^2 - 1/2 * t2^2) * t + 1/2 * (t1 + t2) * t^2 - 1/6 * t^3)
            end
        elseif t3 <= t and t < t4 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = 0
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 + t1 * t3 + t2 * t3)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                                        + (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 + t1 * t3 + t2 * t3) * t)
            end
        elseif t4 <= t and t < t5 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * (-t + t4)
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + t1 * t3 + t2 * t3 + t4 * t - 1/2 * t^2)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 + 1/6 * t4^3 - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                                        + (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + t1 * t3 + t2 * t3) * t
                                        + 1/2 * t4 * t^2 - 1/6 * t^3)
            end
        elseif t5 <= t and t < t6 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * (t4 - t5)
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + t1 * t3 + t2 * t3 + t4 * t - t5 * t)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 + 1/6 * t4^3 - 1/6 * t5^3 - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                                        + (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + t1 * t3 + t2 * t3) * t
                                        + 1/2 * (t4 - t5) * t^2)
            end
        elseif t6 <= t and t <= t7 then
            for axis = 1, Axis do
                ACmd_i[i][axis] = Jerk[axis] * (t + t4 - t5 - t6)
                VCmd_i[i][axis] = Jerk[axis] * (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + 1/2 * t6^2
                                        + t1 * t3 + t2 * t3 + t4 * t - t5 * t - t6 * t + 1/2 * t^2)
                PCmd_i[i][axis] = Pini[axis] + Jerk[axis] * (1/6 * t1^3 + 1/6 * t2^3 + 1/3 * t3^3 + 1/6 * t4^3 - 1/6 * t5^3 - 1/6 * t6^3
                                        - 1/2 * t1 * t3^2 - 1/2 * t2 * t3^2
                                        + (-1/2 * t1^2 - 1/2 * t2^2 - 1/2 * t3^2 - 1/2 * t4^2 + 1/2 * t5^2 + 1/2 * t6^2 + t1 * t3 + t2 * t3) * t
                                        + 1/2 * (t4 - t5 - t6) * t^2 + 1/6 * t^3)
            end
        end

        t = t + sampT
    end

    ret   = 1
    Cmd.P = PCmd_i
    Cmd.V = VCmd_i
    Cmd.A = ACmd_i

    return ret, Cmd
end

function robot.saveData(Cmd, Record, filePath)
    -- Open the file for writing
    print(filePath)
    local file = io.open(filePath, "w")

    local numRows
    local numCols
    if Cmd ~= nil then
        numRows = #Cmd.P
        numCols = #Cmd.P[1]
        print("Cmd row : ", numRows)
        print("Cmd col : ", numCols)
    end
    if Record ~= nil then
        numRows = #Record.P
        numCols = #Record.P[1]
        print("Rec row : ", numRows)
        print("Rec col : ", numCols)
    end
    -- Write the data to the file
    if file then
        for row = 1, numRows do
            for col = 1, numCols do
                file:write(checkNil(Cmd.P[row][col]))
                file:write(" ")
            end
            
            for col = 1, numCols do
                file:write(checkNil(Cmd.V[row][col]))
                file:write(" ")
            end

            for col = 1, numCols do
                file:write(checkNil(Record.P[row][col]))
                file:write(" ")
            end
            
            for col = 1, numCols do
                file:write(checkNil(Record.V[row][col]))
                file:write(" ")
            end

            -- for col = 1, numCols do
            --     file:write(checkNil(Record.TorCtrl[row][col]))
            --     file:write(" ")
            -- end
            file:write("\n")
        end
        -- Close the file
        file:close()
        print("Array saved to:", filePath)
    else
        print("Error opening file for writing.")
    end

end

-- Define the forward kinematics function
function robot.ForwardKinematics(theta)
    -- Because UR5 in coppeliasim isn't the smae with the definition
    local inipose = {pi/2, pi/2, 0, pi/2, 0, 0}
    local theta_vrep = {}
    for i = 1, 6 do
        theta_vrep[i] = theta[i] - inipose[i]
    end

    local A_1 = Tij(1, theta_vrep)
    local A_2 = Tij(2, theta_vrep)
    local A_3 = Tij(3, theta_vrep)
    local A_4 = Tij(4, theta_vrep)
    local A_5 = Tij(5, theta_vrep)
    local A_6 = Tij(6, theta_vrep)

    local T_06 = A_1:mul(A_2):mul(A_3):mul(A_4):mul(A_5):mul(A_6)
    local P = {}
    P[1] = T_06[1][4]
    P[2] = T_06[2][4]
    P[3] = T_06[3][4]
    return mat{P}
end

function robot.PosError(Cmd, Record, type)
    local Error = 0.0 
    if type == "eff_end" then
        Cmd_eff = robot.ForwardKinematics(Cmd.P[#Cmd.P])
        Record_eff = robot.ForwardKinematics(Record.P[#Record.P])
        Error = math.sqrt((Cmd_eff[1][1] - Record_eff[1][1])^2 + (Cmd_eff[1][2] - Record_eff[1][2])^2 + (Cmd_eff[1][3] - Record_eff[1][3])^2)
    elseif type == "eff_all" then
        
    elseif type == "joint_end" then
        for i = 1, #Cmd.P[1] do 
            Error = Error + math.abs(Cmd.P[#Cmd.P][i] - Record.P[#Cmd.P][i])
        end
    elseif type == "joint_all" then
        for j = 1, #Cmd.P do
            for i = 1, #Cmd.P[1] do
                Error = Error + math.abs(Cmd.P[j][i] - Record.P[j][i])
            end
        end
    end
    return Error
end

-- Jacobian mat
function robot.Jacobian(theta)
    local d1 = 0.089159 
    local d4 = 0.10915
    local d5 = 0.09465
    local d6 = 0.0823
    local a2 = -0.425
    local a3 = -0.39225
    local PI = math.pi
    local q1 = theta[1]
    local q2 = theta[2]
    local q3 = theta[3]
    local q4 = theta[4]
    local q5 = theta[5]
    local q6 = theta[6]

    local J = mat{{0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0},
                  {0, 0, 0, 0, 0, 0}}

    J[1][1] =  d6*(sin(PI/2)*(sin(q5)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)))) + cos(q5)*(cos(PI/2)*(cos(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)))) + sin(PI/2)^2*cos(q1))) - cos(PI/2)*(sin(PI/2)*(cos(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)))) - cos(PI/2)*sin(PI/2)*cos(q1))) - a2*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - a3*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - d5*(sin(PI/2)*(cos(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)))) - cos(PI/2)*sin(PI/2)*cos(q1)) + d4*sin(PI/2)*cos(q1)   
    J[1][2] =  d6*(sin(PI/2)*(sin(q5)*(cos(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)))) - cos(PI/2)*cos(q5)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))) + cos(PI/2)*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))) - a2*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) - a3*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + d5*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))
    J[1][3] =  d6*(sin(PI/2)*(sin(q5)*(cos(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)))) - cos(PI/2)*cos(q5)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))) + cos(PI/2)*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))) - a3*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + d5*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))
    J[1][4] =  d6*(sin(PI/2)*(sin(q5)*(cos(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)))) - cos(PI/2)*cos(q5)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))) + cos(PI/2)*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))) + d5*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))))
    J[1][5] = -d6*sin(PI/2)*(cos(q5)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)))) - sin(q5)*(cos(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)))) - sin(PI/2)^2*sin(q1)))
    J[1][6] =  0
    J[2][1] =  d5*(sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)))) + cos(PI/2)*sin(PI/2)*sin(q1)) - d6*(sin(PI/2)*(sin(q5)*(cos(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) - sin(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)))) + cos(q5)*(cos(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)))) - sin(PI/2)^2*sin(q1))) - cos(PI/2)*(sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)) + sin(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1)))) + cos(PI/2)*sin(PI/2)*sin(q1))) + a2*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) + a3*(cos(q3)*(cos(q1)*cos(q2) - cos(PI/2)*sin(q1)*sin(q2)) - sin(q3)*(cos(q1)*sin(q2) + cos(PI/2)*cos(q2)*sin(q1))) + d4*sin(PI/2)*sin(q1)
    J[2][2] =  d6*(sin(PI/2)*(sin(q5)*(cos(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)))) - cos(PI/2)*cos(q5)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))) + cos(PI/2)*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))) - a2*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) - a3*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + d5*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))
    J[2][3] =  d6*(sin(PI/2)*(sin(q5)*(cos(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)))) - cos(PI/2)*cos(q5)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))) + cos(PI/2)*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))) - a3*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + d5*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))
    J[2][4] =  d6*(sin(PI/2)*(sin(q5)*(cos(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)))) - cos(PI/2)*cos(q5)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))) + cos(PI/2)*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))) + d5*sin(PI/2)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))))
    J[2][5] = -d6*sin(PI/2)*(cos(q5)*(cos(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2))) - sin(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)))) - sin(q5)*(cos(PI/2)*(cos(q4)*(cos(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)) + sin(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2))) + sin(q4)*(cos(q3)*(cos(q2)*sin(q1) + cos(PI/2)*cos(q1)*sin(q2)) - sin(q3)*(sin(q1)*sin(q2) - cos(PI/2)*cos(q1)*cos(q2)))) + sin(PI/2)^2*cos(q1))) 
    J[2][6] =  0
    J[3][1] =  0
    J[3][2] =  d6*(cos(PI/2)^2 - 1)*(cos(q2 + q3 + q4)*sin(q5) - cos(PI/2)*sin(q2 + q3 + q4) + cos(PI/2)*sin(q2 + q3 + q4)*cos(q5)) - d5*sin(q2 + q3 + q4)*(cos(PI)/2 - 1/2) + a3*sin(PI/2)*cos(q2 + q3) + a2*sin(PI/2)*cos(q2)
    J[3][3] =  d6*(cos(PI/2)^2 - 1)*(cos(q2 + q3 + q4)*sin(q5) - cos(PI/2)*sin(q2 + q3 + q4) + cos(PI/2)*sin(q2 + q3 + q4)*cos(q5)) - d5*sin(q2 + q3 + q4)*(cos(PI)/2 - 1/2) + a3*sin(PI/2)*cos(q2 + q3)
    J[3][4] =  d6*(cos(PI/2)^2 - 1)*(cos(q2 + q3 + q4)*sin(q5) - cos(PI/2)*sin(q2 + q3 + q4) + cos(PI/2)*sin(q2 + q3 + q4)*cos(q5)) - d5*sin(q2 + q3 + q4)*(cos(PI/2)^2 - 1)
    J[3][5] = -d6*sin(PI/2)*((sin(PI)*sin(q5))/2 + (cos(q2 + q3 + q4)*sin(PI)*sin(q5))/2 + sin(PI/2)*sin(q2 + q3 + q4)*cos(q5))
    J[3][6] =  0
    
    return J
end

return robot