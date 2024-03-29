#!/bin/bash

# How to build your project

# Possibility for cpp agents
# make mainC1 
# make mainC2 
# make mainC3 

# Possibility for java agents
# javac mainC1.java
# javac mainC2.java
# javac mainC3.java

# Possibility for python agents
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt

