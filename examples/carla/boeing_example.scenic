param weather = 'SUNNY'

behavior Wind(init_wind_speed, init_wind_heading):
	wind_speed = init_wind_speed
	wind_heading = init_wind_heading
	speed_perturbation = 1
	heading_perturbation = 1

	while True:
		take generateWindAction(wind_speed, wind_heading)
		wind_speed = Range(wind_speed - speed_perturbation, \
							wind_speed + speed_perturbation)
		wind_heading= Range(wind_heading+heading_perturbation,\
							wind_speed+heading_perturbation)