# -*- coding: utf-8 -*-
"""
Python launcher for interaction with JS
Created on Wed Oct  3 15:35:08 2018

@author: zaira
"""

from bottle import hook, response, route, run, static_file, request
import json
import socket


import transportation
import capacitatedVRP
import cap_tm_wndwVRP

import os

@hook('after_request')
def enable_cors():
    response.headers['Access-Control-Allow-Origin'] = '*'

@route('/sampleJSON/<p>/<n>/<c>', method='GET')
def mySample(p,n,c):
    key = "" #Please Specify your GoggleMapsAPI key inside the quotes 
    if(p=="a"):
        return transportation.runMultiVehicleOptimzation(key,int(n))
    elif(p=="b"):
        return capacitatedVRP.runCapacitatedVRP(key,int(n),int(c))
    elif(p=="c"):
        return cap_tm_wndwVRP.runCapacitated_TW_VRP(key)

run(host='localhost', port=8080)
