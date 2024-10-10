class DetectorConfig:

    default_dict = {    
        "img_size": 48,

        "detect_tl":False,
        "c_quality":23,
        "c_min_R":10,
        "c_max_R":100,
        "c_min_dist":5,
        "mask_iter":5,
        "mask_ks":3,
        # "light_area":0.1,        
        # "red_area":0.2,

        "focal": 500,

        "aruco_w": 7, # см
        "aruco_h": 7, # см

        "traffic_light_radius": 4, # см

        "traffic_sign_radius": 10, # см

        "light_minRadius": 20, 
        "light_maxRadius": 30,

        "lower_red_1": [0,50,50],        
        "upper_red_1": [10,255,255],
        "lower_red_2": [170,50,50],        
        "upper_red_2": [180,255,255],

    #   "lower_red_1": [0,100,100]),
    #   "upper_red_1": [10,255,255]),
    #   "lower_red_2": [160,100,100]),
    #   "upper_red_2": [180,255,255]),

    # "lower_red_1": [0, 120, 70], 
    # "upper_red_1": [10, 255, 255]],
    # "lower_red_2": [170, 120, 70], 
    # "upper_red_2": [180, 255, 255]]

    }

    def __init__(self):
        for key, value in self.default_dict.items():
            setattr(self, key, value)


class RunnerConfig:

    default_dict = {
        "log_file": "robo.log",
        "dist_stop": 5,
        "dist_turn": 5,



        }

    def __init__(self):
        for key, value in self.default_dict.items():
            setattr(self, key, value)