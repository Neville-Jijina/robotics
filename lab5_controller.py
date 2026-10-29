        if state == "FIND_WALL":
                    print("Finding Wall")
                    #right wall threshold
                    #front wall threshold
                    #left wall threshold
                    front = []
                    left = []
                    right = []
            
                    for i, rho in enumerate(lidar_sensor_readings):
                        alpha = lidar_offsets[i]

                        if rho > LIDAR_SENSOR_MAX_RANGE:
                            continue
                            
                        if abs(alpha) <= math.pi / 6:
                            front.append(rho)
                        
                        elif alpha < -math.pi / 6:
                            left.append(rho)
                        
                        else:
                            right.append(rho)
                    
                    front_distance = min(front) 
                    left_distance = min(left)
                    right_distance = min(right)
                    
                    closest_distance = min(front_distance,left_distance,right_distance)

                    if closest_distance == front_distance:
                        if front_distance > wall_distance_thresh:
                            vL = 0.5 * MAX_SPEED
                            vR = 0.5 * MAX_SPEED
                        else:
                            vL = 0
                            vR = 0
                            wall_direction = "LEFT"
                            state = "FOLLOW_WALL"

                    elif closest_distance == left_distance:
                        wall_direction = "LEFT"

                        if left_distance <= wall_detect_thresh:
                            vL = 0
                            vR = 0
                            state = "FOLLOW_WALL"
                        else:
                            vL = -0.25 * MAX_SPEED
                            vR = 0.5 * MAX_SPEED

                    else:
                        wall_direction = "RIGHT"

                        if right_distance <= wall_detect_thresh:
                            vL = 0
                            vR = 0
                            state = "FOLLOW_WALL"
                        else:
                            vL = 0.5 * MAX_SPEED
                            vR = -0.25 * MAX_SPEED
