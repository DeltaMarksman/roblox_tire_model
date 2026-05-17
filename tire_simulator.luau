local tire_simulator = {}

local tire_values = require(script.Parent.tire_values)


local Chassis: Part

function tire_simulator.Initialize(chassis)
	Chassis = chassis
end

local function get_contact_point(suspension_spring: Attachment, tire_values: tire_values.tire)
	-- Raycast
	local raycastParams = RaycastParams.new()
	raycastParams.FilterDescendantsInstances = {script.Parent}
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude
	raycastParams.IgnoreWater = true

	tire_values.contact_raycast = workspace:Raycast(suspension_spring.WorldPosition, suspension_spring.CFrame.UpVector*tire_values.suspension_max_length*-1, raycastParams)
end

local function suspension_step(dt, suspension_spring: Attachment, tire_values: tire_values.tire)
	local distance = if tire_values.contact_raycast then tire_values.contact_raycast.Distance else tire_values.ride_height
	
	-- Spring calculations
	local compression_length 		= tire_values.ride_height - distance
	local suspension_force 			= tire_values.suspension_stiffness * compression_length
	local compressionVelocity 		= (compression_length - tire_values.suspension_compression) / dt
	local damper_force 				= tire_values.suspension_damper * compressionVelocity
	local total_force 				= suspension_force + damper_force
	local total_force				= math.clamp(total_force, -tire_values.suspension_max_force, tire_values.suspension_max_force)
	
	-- Apply force
	if tire_values.contact_raycast then
		Chassis:ApplyImpulseAtPosition(
			tire_values.contact_raycast.Normal * total_force * dt,
			tire_values.contact_raycast.Position
		)
	end
	tire_values.suspension_compression = compression_length
	
	--print("Damping: "..damper_force)
	--print("Total Force: "..total_force)
end

local function friction_step(dt, wheel_hub: Attachment, tire_values: tire_values.tire)
	-- Raycast
	local raycastParams = RaycastParams.new()
	raycastParams.FilterDescendantsInstances = {script.Parent}
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude
	raycastParams.IgnoreWater = true

	local raycastResult = workspace:Raycast(wheel_hub.WorldPosition, wheel_hub.CFrame.UpVector*tire_values.suspension_max_length*-1, raycastParams)
	local distance = if raycastResult then raycastResult.Distance else tire_values.ride_height
	tire_values.is_touching_ground = if raycastResult then true else false
	
	-- Get velocity at contact point if its touching
	if not raycastResult then return end
	local wheelPos = raycastResult.Position
	local wheelVel = Chassis:GetVelocityAtPosition(wheelPos)
	--print(wheelVel)
	
	-- Get wheel directions
	local forward = wheel_hub.WorldCFrame.LookVector
	local right   = wheel_hub.WorldCFrame.RightVector
	
	-- Get friction
	local v_forward = wheelVel:Dot(forward)
	local v_side    = wheelVel:Dot(right)
	
	local longFriction = -v_forward * tire_values.rolling_grip
	local latFriction = -v_side * tire_values.lateral_grip
	
	local frictionForce =
		forward * longFriction +
		right   * latFriction
	
	-- SLIP
	
	
	
	Chassis:ApplyImpulseAtPosition(
		frictionForce * dt,
		raycastResult.Position
	)
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
	print("Steering: "..tire_values.steering_angle)
	print("						Throttle: "..tire_values.throttle)
	print("										Brake: "..brake)
	
	
	
	-- Apply Throttle only to rear wheels
	if wheel_hub.Parent.Name == "LR" or wheel_hub.Parent.Name == "RR" then
		local engineTorque = throttle * tire_values.max_engine_torque
		local forward = wheel_hub.WorldCFrame.LookVector
		local engineForce = math.clamp(engineTorque, -tire_values.longitudinal_grip, tire_values.longitudinal_grip)
		
		if tire_values.contact_raycast and engineForce > 0 then
			print(engineForce) 
			Chassis:ApplyImpulseAtPosition(
				forward * engineForce * dt,
				tire_values.contact_raycast.Position
			)
		end
	end
	
	-- Apply Steering only to Front wheels
	if wheel_hub.Parent.Name == "LF" or wheel_hub.Parent.Name == "RF" then
		wheel_hub.Orientation = Vector3.new(0, tire_values.steering_angle * tire_values.max_steering_angle * -1, 0)
	end
	
	
	-- Apply brake
	if tire_values.contact_raycast then
		local brakeTorque = brake * tire_values.max_brake_torque
		local brakeForce = math.clamp(-brakeTorque, -tire_values.longitudinal_grip, tire_values.longitudinal_grip)
		local forward = wheel_hub.WorldCFrame.LookVector
		local velocity_forward = wheelVel:Dot(forward)
		local brakeForce = 0
		if math.abs(velocity_forward) > 0.05 then
			brakeForce = -math.sign(velocity_forward) * brake * tire_values.max_brake_torque
		end
		
		Chassis:ApplyImpulseAtPosition(
			forward * brakeForce,
			tire_values.contact_raycast.Position
		)
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
		get_contact_point(tire.Suspension_Spring, tire_values[index])
		suspension_step(dt, tire.Suspension_Spring, tire_values[index])
		friction_step(dt, tire.Wheel, tire_values[index])
		control_step(dt, seat, tire.Wheel, tire_values[index])
	end
end

return tire_simulator
