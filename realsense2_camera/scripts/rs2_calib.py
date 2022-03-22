#!/bin/python3

'''
Convert the calibration xml file to yaml file
'''
import ruamel.yaml
import sys
import argparse
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser(description="Converts a cam info into yaml format")
parser.add_argument("--caminfo", dest="caminfo", help="camera info xml file.", required=True)
parser.add_argument("--topic", dest="topic", help="image topic", required=True)
path = parser.parse_args().caminfo
topic = parser.parse_args().topic
path_split = path.split('.')

resolution=[]
distortion=[]
focal_length=[]
principal_point=[]
tree = ET.parse(path)
root = tree.getroot()
for item in root.findall('param'):
	for key,value in item.items():
		if value=='ResolutionRGB':
			for child in item:
				if child.tag == 'value':
					resolution.append(child.text)
		if value=='DistortionRGB':
			for child in item:
				if child.tag == 'value':
					distortion.append(child.text)
		if value=='FocalLengthRGB':
			for child in item:
				if child.tag == 'value':
					focal_length.append(child.text)
		if value=='PrincipalPointRGB':
			for child in item:
				if child.tag == 'value':
					principal_point.append(child.text)


for i in range(0, len(resolution)):
	resolution[i] = int(resolution[i])

for i in range(0, len(distortion)):
	distortion[i] = float(distortion[i])

for i in range(0, len(focal_length)):
	focal_length[i] = float(focal_length[i])

for i in range(0, len(principal_point)):
	principal_point[i] = float(principal_point[i])

d = {
    'cam0': {
        'camera_model': 'pinhole',
        'distortion_coeffs': [distortion[0], distortion[1], distortion[2], distortion[3]],
    	'distortion_model': 'equidistant',
    	'intrinsics': [focal_length[0], focal_length[1], principal_point[0], principal_point[1]],
    	'resolution': [resolution[0], resolution[1]],
    	'rostopic': topic
    }
}

yaml = ruamel.yaml.YAML()
yaml.indent(mapping=2, sequence=4, offset=2)
yaml.version = (1, 2)
yaml.dump(d, sys.stdout)
out_fn = path_split[0] + '.yaml'
outfile = open(out_fn, 'w')
yaml.dump(d, outfile)
