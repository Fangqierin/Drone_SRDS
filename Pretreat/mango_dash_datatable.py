import dash     # need Dash version 1.21.0 or higher
from dash.dependencies import Input, Output, State
import dash_table
import dash_core_components as dcc
import dash_html_components as html
import dash_bootstrap_components as dbc
import dash_bio as dashbio
from dash_bio.utils import PdbParser, create_mol3d_style
from dash.dependencies import Input, Output
import pandas as pd
from collections import defaultdict
import plotly.express as px
import matplotlib.pyplot as plt
import mpld3 
import pymongo
from pymongo import MongoClient
from bson import ObjectId
from fire_sim import Sim_fire2
import random
# Connect to local server
client = MongoClient("mongodb://127.0.0.1:27017/")
# Create database called animals
mydb = client["Command"]
# Create Collection (table) called shelterA
collection = mydb.firefighter

record = {
    "Area": 1,
    "Time": 0,
    "Event": "Fire",
    "Frequency (TPM)": 2,
    "Sign.": 2,
}

collection_fire=client['Fire_setting'].status

colors = {
    'background': '#F0FBFA',
    'text': '#B41907'
}

app = dash.Dash(__name__, suppress_callback_exceptions=True,
                external_stylesheets=['https://codepen.io/chriddyp/pen/bWLwgP.css'])
parser = PdbParser('https://git.io/4K8X.pdb')
data = parser.mol3d_data()
#print(data)
#x,y,z=list(range(10))

styles = create_mol3d_style(
    data['atoms'], visualization_type='cartoon', color_element='residue'
)

app.layout = html.Div(style={'backgroundColor': colors['background']},children=[
    html.H1(children='DragonFly Dashboard', style={
        'textAlign': 'center',
        'color': colors['text']}),
    html.Div(children='''Tasks for drones.'''),
    html.Div(id='mongo-datatable', children=[]),
    # activated once/week or when page refreshed
    dcc.Interval(id='interval_db', interval=86400000 * 7, n_intervals=0),
    html.Button("Submit Tasks", id="save-it"),
    html.Button('Add Task', id='adding-rows-btn', n_clicks=0),
    html.Div(children='''High-rise status.'''),
    html.Div(id='fire-datatable', children=[]),
    #html.Div(children='''Task statistic.'''),
    html.Div(id="show-graphs", children=[]),
    html.Div(id="drone-graphs", children="Status of Drones"),
    html.Div(id="placeholder"),
])
# Display Datatable with data from Mongo database *************************
@app.callback(Output('mongo-datatable', 'children'),
              [Input('interval_db', 'n_intervals')])
def populate_datatable(n_intervals):
    
    client = MongoClient("mongodb://127.0.0.1:27017/")
    # Create database called animals
    mydb = client["Command"]
    # Create Collection (table) called shelterA
    collection = mydb.wins
    #windows=pd.read_csv('all_win.csv', delim_whitespace=True)
    windows=pd.read_csv('../data/all_win.csv',sep=' ')

    #print(windows)
    collection.delete_many({})
    for index, row in windows.iterrows():
            #print(windows.columns)
    #         print(row._get_value('id'))
    #         print(row['id'])
        record={"id":row["id"], "x":row["v_1_x"], "y":row["v_1_y"], 
              "z":row["v_1_z"], 'win': 'Close', 'fire': 'None', 'hum': 'None'}
        collection.insert_one(record)

    # Convert the Collection (table) date to a pandas DataFrame
    df = pd.DataFrame(list(collection.find()))
    #Drop the _id column generated automatically by Mongo
    df = df.iloc[:, 1:]
    return [
        dash_table.DataTable(
            id='my-table',
            columns=[{
                'name': x,
                'id': x,
            } for x in df.columns],
            data=df.to_dict('records'),
            editable=True,
            row_deletable=True,
            filter_action="native",
            #filter_options={"case": "sensitive"},
            sort_action="native",  # give user capability to sort columns
            sort_mode="single",  # sort across 'multi' or 'single' columns
            page_current=0,  # page number that user is on
            page_size=20,  # number of rows visible per page
            style_table={'overflowX': 'scroll', 'maxHeight': '300px', 'overflowY':'scroll'},
            style_cell={'textAlign': 'left', 'minWidth': '0','width': '3',
                         'maxWidth': '0','overflow': 'hidden',
        'textOverflow': 'ellipsis',},
            style_data={ 'whiteSpace': 'normal', 'height': 'auto', 'lineHeight': '15px'},),
        html.H4('High-rise Building Overview'),
        html.Div(children=[
        #
        dcc.Graph(id="graph1",style={'display': 'inline-block'}),
        #html.H4('Simulation'),
        dcc.Graph(id="graph2",style={'display': 'inline-block'}),]),
        html.P("Fire Simulation Speed:"),
        dcc.RangeSlider(
            id='Time',
            min=1, max=5, step=1,
            marks={1: '*1', 5: '*5'},
            value=[1, 5])
        ]

@app.callback(Output('fire-datatable', 'children'),
              [Input('interval_db', 'n_intervals')])

def populate_fire(n_intervals):
    #print(n_intervals)
    # Convert the Collection (table) date to a pandas DataFrame
    df = pd.DataFrame(list(collection_fire.find()))
    #Drop the _id column generated automatically by Mongo
    df = df.iloc[:, 1:]
    #print(df.head(6))
    return [
        dash_table.DataTable(
            id='my-fire',
            columns=[{
                'name': x,
                'id': x,
            } for x in df.columns],
            data=df.to_dict('records'),
            editable=True,
            row_deletable=True,
            filter_action="native",
            #filter_options={"case": "sensitive"},
            sort_action="native",  # give user capability to sort columns
            sort_mode="single",  # sort across 'multi' or 'single' columns
            page_current=0,  # page number that user is on
            page_size=10,  # number of rows visible per page
            style_cell={'textAlign': 'left', 'minWidth': '0','width': '2',
                         'maxWidth': '0','overflow': 'hidden',
        'textOverflow': 'ellipsis',},
            style_data={ 'whiteSpace': 'normal', 'height': 'auto', 'lineHeight': '12px'},
            style_table={'overflowX': 'scroll', 'overflowY':'scroll'})
    ]

# Add new rows to DataTable ***********************************************
@app.callback(
    Output('my-table', 'data'),
    [Input('adding-rows-btn', 'n_clicks')],
    [State('my-table', 'data'),
     State('my-table', 'columns')],
)
def add_row(n_clicks, rows, columns):
    if n_clicks > 0:
        #print(f"see click {n_clicks}")
        #print(f"why???? {[c['id'] for c in columns]}")
        rows.append({c['id']: '' for c in columns})
        
    return rows
# Save new DataTable data to the Mongo database ***************************
@app.callback(
    Output("placeholder", "children"),
    Input("save-it", "n_clicks"),
    State("my-table", "data"),
    prevent_initial_call=True
)
def save_data(n_clicks, data):
    dff = pd.DataFrame(data)
    collection.delete_many({})
    collection.insert_many(dff.to_dict('records'))
    #print(f"Saved to MongoDB")
    return " "

@app.callback(
    Output("graph1", "figure"), 
    Input("Time", "value"))
def update_bar_chart(Time):
    #print(f"slider_range {slider_range}")
    client = MongoClient("mongodb://127.0.0.1:27017/")
    # Create database called animals
    mydb = client["Command"]
    # Create Collection (table) called shelterA
    collection = mydb.wins
    low, high = Time
    print(f"check time {Time}")
    Fireset=Sim[low]
    for f in Fireset['f']:
        #print(f'see check f {f}')
        #print(f"ccc", df.loc(f))
        myquery = { "id": f }
        w=collection.find(myquery)[0]
        #print(f"check find {w}")
        w['fire']='Burn'
        collection.update_one(myquery,{"$set": w})     
    df = pd.DataFrame(list(collection.find()))
    fig = px.scatter_3d(df, x='x', y='y', z='z', color='fire',hover_data=['hum'])
    #fig2 = px.scatter_3d(df, x='x', y='y', z='z', color='hum',hover_data=['hum'])
    #,color="species", ])
    return fig


@app.callback(
    Output("graph2", "figure"),
    Input("Time", "value"))
def update_bar_chart2(Time):
    #print(f"slider_range {slider_range}")
    client = MongoClient("mongodb://127.0.0.1:27017/")
    # Create database called animals
    mydb = client["Command"]
    # Create Collection (table) called shelterA
    collection = mydb.Simfire
    low, high = Time
    
    print(f"check time {Time}")
    #Fireset=Sim[low]
#     for f in Fireset['f']:
#         #print(f'see check f {f}')
#         #print(f"ccc", df.loc(f))
#         myquery = { "id": f }
#         w=collection.find(myquery)[0]
#         #print(f"check find {w}")
#         w['fire']='Burn'
        #collection.update_one(myquery,{"$set": w})     
    df = pd.DataFrame(list(collection.find()))
    #df=df.query('time % 2==0')
    #print(f"check {df} {df[df['time'] in time]}")
    fig = px.scatter_3d(df, x='x', y='y', z='z', color='fire', animation_frame='time',hover_data=['hum'])
    #fig2 = px.scatter_3d(df, x='x', y='y', z='z', color='hum',hover_data=['hum'])
    #,color="species", ])
    return fig


def Get_sim(st,et, rseed,a,output,time_slot):  # here a is a window!!!!! 
    lay_num=12
    Sim_fire2(st,et,rseed,a,output,time_slot)  # write the simulation in a file.
    Timeline=pd.read_csv(output,sep='/ ',engine='python')
    Sim=defaultdict(dict)
    Sim_real=defaultdict(dict)
    win=[]
    firs=list(Timeline['f'])
    tmp=[i[1:-1].split(', ') for i in firs]
    all_fire=set()
    for i in tmp:
        try:
            all_fire.update([int(k) for k in i])
        except:
            pass
    hum_floors=set([i//lay_num for i in all_fire]+[i//lay_num +1 for i in all_fire])
    fire_floors=set([i//lay_num for i in all_fire])  
    win_floors=set([i//lay_num for i in all_fire]+[i//lay_num -1 for i in all_fire])  # this layer and the lower layer. 
    #print(f"ss {all_floors}")
    for index,row in Timeline.iterrows():
        #print(row)
        tmp=row['f'][1:-1].split(', ')
        tmps=row['f_s'][1:-1].split(', ')
        #print(f"check  {tmp}")
        if len(tmp)==1 and tmp[0]=='':
            Sim[row['t']]['f']=[]
            Sim_real[row['t']]['f']=[]
            Sim[row['t']]['f_s']=[]
            Sim_real[row['t']]['f_s']=[]
        else:
            fire=[int(tmp[i]) for i in range(len(tmp))]
            fire_state=[int(tmps[i]) for i in range(len(tmp))]
            Sim[row['t']]['f']=fire
            Sim[row['t']]['f_s']=fire_state
            Sim_real[row['t']]['f']=fire
            Sim_real[row['t']]['f_s']=fire_state
        tmp=row['w'][1:-1].split(', ')
        #if len(tmp)>0:
        winn=[int(tmp[i]) for i in range(len(tmp))]
        win=[]
        for i in winn:
            if i//lay_num in win_floors: # 
                win.append(i)
        Sim[row['t']]['w']=win
        Sim_real[row['t']]['w']=win
        #except:
            #Sim[row['t']]['w']=[]
        tmp=row['h'][1:-1].split(', ')
        tmps=row['h_s'][1:-1].split(', ')
        try:
            humm=[int(tmp[i]) for i in range(len(tmp))]
            hum_s=[int(tmps[i]) for i in range(len(tmp))]
            h_s=dict(zip(humm,hum_s))
            hum=[];state=[]
            for i in humm:
                if i//lay_num in hum_floors:
                    hum.append(i)
                    state.append(h_s.get(i))
            Sim[row['t']]['h']=hum 
            Sim[row['t']]['h_s']=state
            Sim_real[row['t']]['h']=hum
            Sim_real[row['t']]['h_s']=state
        except:
            Sim[row['t']]['h']=[]
            Sim[row['t']]['h_s']=[]
            Sim_real[row['t']]['h_s']=[]
            Sim_real[row['t']]['h']=[]     
    return Sim,Sim_real,fire_floors, hum_floors, win_floors

if __name__ == '__main__':
    floor_num=12
    va=3
    rooms_d=pd.read_csv('../data/rooms.csv',sep=',')
    #room_set=list(rooms_d['r'])
    room_AF=dict(zip(list(rooms_d['r']),(zip(list(rooms_d['w']),list(rooms_d['d'])))))
    all_room=len(room_AF)*floor_num
    ii=0
    time_slot=1
    random.seed(ii)
    a=random.sample(range(all_room),va) #fire source 
    #print(f"fire source",a)
    output=f"./result/dash/sim_{ii}.csv"
    Sim,Sim_real,fire_floors,hum_floors,win_floors=Get_sim(0,60,ii,a,output,time_slot)
    client = MongoClient("mongodb://127.0.0.1:27017/")
    # Create database called animals
    mydb = client["Command"]
    # Create Collection (table) called shelterA
    collection = mydb.Simfire
    windows=pd.read_csv('../data/all_win.csv',sep=' ')
    collection.delete_many({})
    for key, item in Sim.items():
        for index, row in windows.iterrows():
                #print(windows.columns)
        #         print(row._get_value('id'))
        #         print(row['id'])
                if row["id"]in item['f']:
                    record={"time": key, "id":row["id"], "x":row["v_1_x"], "y":row["v_1_y"], 
                          "z":row["v_1_z"], 'win': 'Close', 'fire': 'Burn', 'hum': 'None'}
                else: 
                    record={"time": key, "id":row["id"], "x":row["v_1_x"], "y":row["v_1_y"], 
                          "z":row["v_1_z"], 'win': 'Close', 'fire': 'None', 'hum': 'None'}
                collection.insert_one(record)
    print(f"See Sim {Sim}")
    
    app.run_server(debug=True)
