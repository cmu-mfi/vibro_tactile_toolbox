#!/usr/bin/env python3

import argparse
import rospy
import yaml
from test.check_ros_topics import check_ros_topics, check_ros_services

def main(args):
    rospy.init_node('test_vibro_tactile_system', anonymous=True)

    if args.type == 'nist':
        yaml_file = 'nist.yaml'
    elif args.type == 'lego':
        yaml_file = 'lego.yaml'

    with open('config/'+yaml_file) as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as error:
            print(error)

    for key in config.keys():
        if isinstance(config[key], dict):
            if 'namespace' in config[key]:
                config[key]['namespace'] = args.namespace
            if 'topic_name' in config[key]:
                config[key]['topic_name'] = config[key]['topic_name'].replace("namespace", args.namespace)
        if isinstance(config[key], list):
            for i in range(len(config[key])):
                if isinstance(config[key][i], str) and 'namespace' in config[key][i]:
                    config[key][i] = config[key][i].replace("namespace", args.namespace)

    topics = []

    for topic in config['rosbag_data_recorder']['topics']:
        if 'namespace' in topic:
            topic = topic.replace("namespace", args.namespace)
        topics.append(topic)

    check_ros_topics(topics)
    check_ros_services(config['ros_services'])

if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('-n', '--namespace', type=str,
      help='Namespace to use')
    args.add_argument('-t', '--type', type=str,
      help='Type of System (lego or nist)')
    args = args.parse_args()

    main(args)
