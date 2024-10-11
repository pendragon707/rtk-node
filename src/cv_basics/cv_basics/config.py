class DetectorConfig:

    default_dict = {    
        "img_size": 48,
                
        "mask_iter":5,
        "mask_ks":3,

        "focal_length": 50,        
        "aruco_perimeter": 50,
        "traffic_sign_perimeter": 314,
        "traffic_light_perimeter": 126,        

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