import configparser

config = configparser.ConfigParser()
config.read('config.ini')

print((config.get('MicroXRCEAgent', 'cmd').split()))