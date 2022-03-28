# Run this app with `python app.py` and
# visit http://192.168.0.102:8050/ in your web browser.

########## Imports ############

from distutils.log import debug
from re import template
import dash
from dash import html
from dash import dcc
import dash_daq as daq
import serial
import time, math
import threading
import json
import plotly.graph_objs as go
import plotly
import logging
from math import sqrt

########## Global variables ##########

# scatter plot vars
X = [0]
Y = [0]
position = 0

# barchart vars
prox_sensor_names = ['Left-Left', 'Left', 'Center', 'Right', 'Right-Right']
line_sensor_values = [0] * 5

# direction discovery vars
clockwise_arr = [0] * 400
counter_clockwise_arr = [0] * 400
counter = 0

# joystick vars
zumoAngle = 0
zumoSpeed = 0

# button vars
auto_flag = 0
reset_plot = False
show_last = False

# Slider vars
Kp = 0.2
Kd = 0
Ki = 0
Speed = 80

# computations vars
average = 0
sd = 0
out_of_bounds_counter = 0
accuracy_rate = 0
DT = 0
timestamp = 0
velocity = 0

########## Serial Connection ##########

ser = serial.Serial('/dev/ttyACM0')  # open serial port
#ser = serial.Serial('COM3', baudrate = 9600, timeout = 1)

# App Initialization and definition

app = dash.Dash(__name__)

app.layout = html.Div(children=[daq.Joystick(id='my-joystick', label="Zumo Joystick", angle=0, size=100),
                                html.Div(id='joystick-output'),
                                html.Button('Automatic', id='auto_button', n_clicks=0),
                                html.Button('Manual', id='manual_button', n_clicks=0),
                                html.Div(id='Button output'),
                                html.Div([dcc.Slider(80, 200, 20, value=80, id='Speed-slider', marks=None, tooltip={"placement": "bottom", "always_visible": True}),
                                          html.Div(id='Speed-slider-output-container')]),
                                html.Div([dcc.Slider(0.2, 2.5, 0.1, value=0.2, id='Kp-slider', marks=None, tooltip={"placement": "bottom", "always_visible": True}),
                                          html.Div(id='Kp-slider-output-container')]),
                                html.Div([dcc.Slider(0, 100, 1, value=0, id='Kd-slider', marks=None, tooltip={"placement": "bottom", "always_visible": True}),
                                          html.Div(id='Kd-slider-output-container')]),
                                html.Div([dcc.Slider(0, 2, 0.1, value=0, id='Ki-slider', marks=None, tooltip={"placement": "bottom", "always_visible": True}),
                                          html.Div(id='Ki-slider-output-container')]),
                                html.Button('Reset Plot', id='reset_button', n_clicks=0),
                                html.Button('Show last 500 samples', id='show_last_button', n_clicks=0),
                                html.Button('Show all samples', id='show_all_button', n_clicks=0),
                                html.Div(id='plot_buttons_output'),
                                html.Div([html.Div(id='computations_output'),
                                          dcc.Interval(id = 'computations-update', interval = 100, n_intervals = 0)]),
                                html.Div([dcc.Graph(id = 'live-graph', animate = True),
                                          dcc.Interval(id = 'graph-update', interval = 700, n_intervals = 0)]),
                                html.Div([dcc.Graph(id = 'live-barchart', animate = True),
                                          dcc.Interval(id = 'barchart-update', interval = 700, n_intervals = 0)])
])

########## Callback Functions ##########

# Callback function - Speed slider
@app.callback(dash.dependencies.Output('Speed-slider-output-container', 'children'),
              dash.dependencies.Input('Speed-slider', 'value'))
def update_output(value):
    global Speed
    Speed = value
    return 'Speed = {}'.format(value)

# Callback function - Kp slider
@app.callback(dash.dependencies.Output('Kp-slider-output-container', 'children'),
              dash.dependencies.Input('Kp-slider', 'value'))
def update_output(value):
    global Kp
    Kp = value
    return 'Kp = {}'.format(value)

# Callback function - Kd slider
@app.callback(dash.dependencies.Output('Kd-slider-output-container', 'children'),
              dash.dependencies.Input('Kd-slider', 'value'))
def update_output(value):
    global Kd
    Kd = value
    return ['Kd = {}'.format(value), html.Br(), html.Br()]

# Callback function - Ki slider
@app.callback(dash.dependencies.Output('Ki-slider-output-container', 'children'),
              dash.dependencies.Input('Ki-slider', 'value'))
def update_output(value):
    global Ki
    Ki = value / 1000.0
    return ['Ki = {} x 10^-3'.format(value), html.Br(), html.Br()]

# Callback function - Computations
@app.callback(dash.dependencies.Output('computations_output', 'children'),
              dash.dependencies.Input('computations-update', 'n_intervals'))

def computations_output(n):
  global average, sd, accuracy_rate, velocity
  return 'Average = {}, SD = {}, Accuracy rate = {}%, Velocity = {} cm/s'.format(average, sd, accuracy_rate, round(velocity, 2))

# Callback function - Scatter plot
@app.callback(dash.dependencies.Output('live-graph', 'figure'),
              dash.dependencies.Input('graph-update', 'n_intervals'))

def update_graph_scatter(n):
    global reset_plot, X, Y, show_last
    if reset_plot == True:
      X = [0]
      Y = [0]
      reset_plot = False

    data = go.Scatter(x=list(X), y=list(Y), name='Scatter', mode='lines+markers')
    if show_last == True:
      xaxis=dict(range=[max((0, max(X)-500)), max(X)])
    else:
      xaxis=dict(range=[0, max(X)])

    return {'data': [data],
            'layout': go.Layout(xaxis=xaxis, # last 500 samples: range=[max((0, max(X)-500)), max(X)]
                                yaxis=dict(range=[0, 4000]))}
#                                shapes={'type': 'line', 'xref': 'x', 'yref': 'y', 'x0': max((0, max(X)-500)), 'y0': 2000, 'x1': max(X), 'y1': 2000})}

# Callback function - Barchart
@app.callback(dash.dependencies.Output('live-barchart', 'figure'),
              [dash.dependencies.Input('barchart-update', 'n_intervals')])

def update_graph_bar(n):
  clrs = list()
  colorred = 'rgb(222,0,0)'
  colorgreen = 'rgb(0,222,0)'
  for i in range(len(line_sensor_values)):
    if (line_sensor_values[i] > 200):
      clrs.append(colorred)
    else:
      clrs.append(colorgreen)

  traces = list()
  traces.append(plotly.graph_objs.Bar(x=prox_sensor_names, y=line_sensor_values, name='Bar', marker=dict(color=clrs)))
  layout = go.Layout(barmode='group', yaxis=dict(range=[0, 1000]))
  if traces is not None and layout is not None:
    return {'data': traces, 'layout': layout}

# Callback function - Mode button clicks
@app.callback(dash.dependencies.Output('Button output', 'children'),
              [dash.dependencies.Input('auto_button', 'n_clicks'),
              dash.dependencies.Input('manual_button', 'n_clicks')])

def displayClick(auto_button, manual_button):
  global auto_flag, clockwise_arr, counter_clockwise_arr, out_of_bounds_counter
  msg = 'Manual mode'
  changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]
  if 'auto_button' in changed_id:
    msg = 'Auto mode'
    auto_flag = 1
    out_of_bounds_counter = 0
    clockwise_arr = [0] * 400
    counter_clockwise_arr = [0] * 400
  elif 'manual_button' in changed_id:
    msg = 'Manual mode'
    auto_flag = 0

  return [msg, html.Br(), html.Br()]

# Callback function - Plot buttons
@app.callback(dash.dependencies.Output('plot_buttons_output', 'children'),
              [dash.dependencies.Input('reset_button', 'n_clicks'),
              dash.dependencies.Input('show_last_button', 'n_clicks'),
              dash.dependencies.Input('show_all_button', 'n_clicks')])

def plot_buttons(reset_button, show_last_button, show_all_button):
  global reset_plot, show_last
  msg = 'Showing all samples'
  changed_id_2 = [p['prop_id'] for p in dash.callback_context.triggered][0]
  if 'reset_button' in changed_id_2:
    reset_plot = True
  if 'show_last_button' in changed_id_2:
    show_last = True
    msg = 'Showing last 500 samples'
  elif 'show_all_button' in changed_id_2:
    show_last = False
    msg = 'Showing all samples'

  return msg

# Callback function - Joystick
@app.callback(
    dash.dependencies.Output('joystick-output', 'children'),
    [dash.dependencies.Input('my-joystick', 'angle'),
     dash.dependencies.Input('my-joystick', 'force')])

def update_output(angle, force=0):
    #print("update_output")
    global zumoSpeed, zumoAngle
    if force is None:
        force = 0
    if type(angle) == int or float:
      zumoAngle = angle
    else:
      zumoAngle = 0

    if type(force) == int or float:
      zumoSpeed = force
    else:
      zumoSpeed = 0

    return ['Angle is {}'.format(round(angle, 2)),
            html.Br(),
            'Force is {}'.format(zumoSpeed),
            html.Br(), html.Br()]

# Entry point of the transmit thread
def TransmitThread():
  #print ("transmitThread")
  global control_params, clockwise_arr, counter_clockwise_arr, Kp, Kd, Ki
  while ser.isOpen:
    global zumoSpeed, zumoAngle, auto_flag
    if zumoAngle is None:
      zumoAngle = 0
    if zumoSpeed is None:
      zumoSpeed = 0
    joyY = math.sin(math.radians(zumoAngle))*zumoSpeed*200
    joyX = math.cos(math.radians(zumoAngle))*zumoSpeed*200
    if sum(counter_clockwise_arr) > sum(clockwise_arr):
      direction = '0'
    else:
      direction = '1'
    msg = ''
    msg = str(auto_flag) + ';' + direction + '.' + str(joyX) + ',' + str(joyY) + '?' + str(Kp) + '!' + str(Kd) + '@' + str(Ki) + '#' + str(Speed) + '\r\n'
    ser.write(msg.encode('ascii'))
    time.sleep(0.04)

# Entry point of the receive thread
def ReceiveThread():
  global line_sensor_values, clockwise_arr, counter_clockwise_arr, counter, position, X, Y, average, sd, out_of_bounds_counter, accuracy_rate, DT, timestamp, velocity
  while(ser.isOpen):
    if (ser.in_waiting > 0):
      sensor = ser.readline().decode('ascii')
      data = json.loads(sensor)
      position = data["Position"]
      velocity = data["Velocity"]
      if auto_flag == 1:
        X.append(X[-1]+1)
        Y.append(position)
        if len(Y) > 10:
          if (position < 1960 or position > 2040):
            out_of_bounds_counter += 1
          accuracy_rate = round(100 * float((len(Y[10:]) - out_of_bounds_counter)) / len(Y[10:]), 2)
        
        average = ( round( (float(sum(Y[10:])) / len(Y[10:])), 2 ) ) if len(Y) > 10 else 0
        sd = round( sqrt( float( sum( [ (position-average)**2 for position in Y[10:] ] ) ) / len(Y[10:]) ), 2) if len(Y) > 10 else 0
        DT = data["DT"]
        timestamp = data["timestamp"]
        
        #print('DT = {}, timestamp = {}'.format(str(DT), str(timestamp)))
        #print('Velocity = {} cm/s, left_velocity = {}, right_velocity = {}'.format(data["Velocity"], data["left_velocity"], data["right_velocity"]))
        #print ('sum = {}, len = {}, average = {}, sd = {}'.format(sum(X[10:]), len(X[10:]), average, sd))
      for i in range(len(data["line_sensor_values"])):
        line_sensor_values[i] = data["line_sensor_values"][i]
      counter_clockwise_arr[counter%400] = line_sensor_values[0] + line_sensor_values[1]
      clockwise_arr[counter%400] = line_sensor_values[3] + line_sensor_values[4]
      counter += 1
    else:
      time.sleep(0.02)

# Entry point of the app thread
def start_app():
  #app.run_server(debug=True, host='0.0.0.0')
  app.server.run(port=8050, host='0.0.0.0')

def initializeThreads():
  # supress logging
  log = logging.getLogger('werkzeug')
  log.setLevel(logging.ERROR)
  
  t1 = threading.Thread(target=start_app)
  t2 = threading.Thread(target=ReceiveThread)
  t3 = threading.Thread(target=TransmitThread)   
  t1.start()
  t2.start()
  t3.start()
  t1.join()
  t2.join()
  t3.join()

if __name__ == "__main__":
  initializeThreads()
