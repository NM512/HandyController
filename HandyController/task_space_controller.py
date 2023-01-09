import os
import numpy
import ikpy.chain as chain
import ikpy.link as link
import yaml


class ShadowHandIK():
    def __init__(self):
        file_path = os.path.dirname(__file__)
        shadow_info_path = os.path.join(file_path, './cfg/shadowhand_info.yaml')
        with open(shadow_info_path, 'r') as yml:
            self.shadow_cfg = yaml.safe_load(yml)

        urdf_path = os.path.join(file_path, './urdf/.yaml')

        self.chains = {}
        for finger in self.fingers.keys():
            self.chains[finger] = chain.Chain.from_urdf_file(urdf_path, base_elements = [self.cfg.links_info['base']['link'], self.cfg.links_info[finger]['link']], name = finger)