local tire_simulator = {}

local tire_values = require(script.Parent.tire_values)


local Chassis: Part

function tire_simulator.Initialize(chassis)
	Chassis = chassis
end

local function get_contact_point(suspension_spring: Attachment, contact_point: Attachment, tire_values: tire_values.tire)
	-- Raycast
	local raycastParams = RaycastParams.new()
	raycastParams.FilterDescendantsInstances = {script.Parent}
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude
	raycastParams.IgnoreWater = true

	tire_values.contact_raycast = workspace:Raycast(suspension_spring.WorldPosition, suspension_spring.CFrame.UpVector*tire_values.suspension_max_length*-1, raycastParams)
	local distance = if tire_values.contact_raycast then tire_values.contact_raycast.Distance else tire_values.ride_height

	-- Set attachment for contact point
	contact_point.Position = suspension_spring.Position + Vector3.new(0, -distance, 0)
end

local function suspension_step(dt, suspension_spring: Attachment, contact_point: Attachment, tire_values: tire_values.tire)
	local distance = if tire_values.contact_raycast then tire_values.contact_raycast.Distance else tire_values.ride_height
	local vector_force: VectorForce = contact_point.SpringForce

	-- Spring calculations
	local compression_length 		= tire_values.ride_height - distance
	local suspension_force 			= tire_values.suspension_stiffness * compression_length
	local compressionVelocity 		= (compression_length - tire_values.suspension_compression) / dt
	local damper_force 				= tire_values.suspension_damper * compressionVelocity
	local total_force 				= suspension_force + damper_force
	local total_force				= math.clamp(total_force, 0, tire_values.suspension_max_force)

	-- Limit spring speed
	if math.abs(total_force - tire_values.suspension_force) > 100 then
		total_force += math.sign(total_force - tire_values.suspension_force) * 100
	end

	-- Apply force
	if tire_values.contact_raycast then
		--transform normal relative to contact point
		vector_force.Force = tire_values.contact_raycast.Normal * total_force * dt
	else
		vector_force.Force = Vector3.new()
	end
	tire_values.suspension_compression  = compression_length
	tire_values.suspension_force		= total_force

	--print("Damping: "..damper_force)
	--print("Total Force: "..total_force)
end


local function slip_step(dt, wheel_hub: Attachment, contact_point: Attachment, tire_values: tire_values.tire)
	local vector_force: VectorForce = contact_point.FrictionForce
	vector_force.Force = Vector3.new()
	if not tire_values.contact_raycast then 
		return
	end


	local wheelPos = tire_values.contact_raycast.Position
	local wheelVel = Chassis:GetVelocityAtPosition(wheelPos)
	local wheelSpeed = wheelVel.Magnitude

	local forward = wheel_hub.WorldCFrame.LookVector
	local right   = wheel_hub.WorldCFrame.RightVector

	local v_long = wheelVel:Dot(forward)
	local v_lat  = wheelVel:Dot(right)

	-- slip = (R*ω - Vx) / |Vx|
	local slipRatio = (tire_values.angular_velocity - v_long) / math.abs(v_long)
	slipRatio = math.clamp(slipRatio, -1, 1)


	-- slip angle
	local slipAngle = math.atan2(v_lat, math.abs(v_long) + 1)


	-- slip forces (TODO update to pacejka)
	local Fx = tire_values.longitudinal_grip * slipRatio / (1 + math.abs(slipRatio))
	local Fy = tire_values.lateral_grip * slipAngle / (1 + math.abs(slipAngle))

	-- Combined slip normalization
	local FxMax = tire_values.longitudinal_grip
	local FyMax = tire_values.lateral_grip

	local combined = math.sqrt((Fx*Fx)/(FxMax*FxMax) + (Fy*Fy)/(FyMax*FyMax))
	if combined > 1 then
		Fx /= combined
		Fy /= combined
	end

	--speed
	local speedFactor = 1 / (1 + wheelSpeed / 5000)
	Fx *= speedFactor
	Fy *= speedFactor


	--weight
	local load = tire_values.suspension_force
	local loadFactor = math.sqrt(math.max(load / 500, 0))
	Fx *= loadFactor
	Fy *= loadFactor

	--apply force
	local force = forward * Fx + right * -Fy

	if tire_values.contact_raycast and Fx == Fx and Fy == Fy then
		vector_force.Force = force
	end

	--update rotation
	local tireTorque = -Fx
	local brakeTorque = tire_values.brake * tire_values.max_brake_torque
	local rollingFrictionTorque = tire_values.rolling_grip * -math.sign(v_long)

	-- Angular acceleration
	local alpha = (tireTorque + rollingFrictionTorque)

	-- Integrate wheel angular velocity
	tire_values.angular_velocity += alpha * dt


	print(wheel_hub.Parent.Name .. " -> "..v_long.."\t|\t "..tire_values.angular_velocity.."\t|\t "..Fx)
end


local function control_step(dt, seat:VehicleSeat, wheel_hub: Attachment, tire_values: tire_values.tire)
	-- Get keyboard
	local throttle 		= 0
	local brake			= 0

	-- Throttle
	if game:GetService("UserInputService"):IsKeyDown(Enum.KeyCode.W) then
		throttle = 1
	else
		throttle = 0
	end

	-- Brake
	if game:GetService("UserInputService"):IsKeyDown(Enum.KeyCode.S) then
		brake = 1
	else
		brake = 0
	end


	local wheelVel = Chassis:GetVelocityAtPosition(wheel_hub.WorldPosition)
	local wheelSpeed = wheelVel.Magnitude
	tire_values.steering_angle 	+= (seat.SteerFloat - tire_values.steering_angle)*tire_values.keyboard_damp*dt / math.max(1,wheelSpeed*tire_values.keyboard_speed_damp)
	tire_values.throttle 		+= (throttle - tire_values.throttle)*tire_values.keyboard_damp*dt
	tire_values.brake	 		+= (brake - tire_values.brake)*tire_values.keyboard_damp*dt
	--print("Steering: "..tire_values.steering_angle)
	--print("						Speed Goal: "..tire_values.engine_speed)



	-- Apply Steering only to Front wheels
	if wheel_hub.Parent.Name == "LF" or wheel_hub.Parent.Name == "RF" then
		--wheel_hub.Orientation = Vector3.new(0, tire_values.steering_angle * tire_values.max_steering_angle * -1, 0)
	end

	if wheel_hub.Parent.Name == "LR" or wheel_hub.Parent.Name == "RR" then
		-- Rear wheels get engine torque
		tire_values.engine_speed = math.max(math.min(tire_values.engine_speed + 10000*dt*throttle - 5000*dt  - tire_values.max_brake_torque*brake*dt, tire_values.max_engine_torque), 0)
	else

	end
end


function tire_simulator.Step(dt, seat)
	local tires = {
		Chassis.Tires.LF,
		Chassis.Tires.RF,
		Chassis.Tires.LR,
		Chassis.Tires.RR,
	}

	for index, tire: Folder in tires do
		get_contact_point(tire.Suspension_Spring, tire.Contact_Point, tire_values[index])
		suspension_step(dt, tire.Suspension_Spring, tire.Contact_Point, tire_values[index])
		slip_step(dt, tire.Wheel, tire.Contact_Point, tire_values[index])
		control_step(dt, seat, tire.Wheel, tire_values[index])
	end
end

return tire_simulator
