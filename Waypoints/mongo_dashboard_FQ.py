
'''
This file can be executed to launch our mongodb database
'''

from gc import callbacks
from jupyter_dash import JupyterDash     # need Dash version 1.21.0 or higher
from dash.dependencies import Input, Output, State
import dash_table
import dash_core_components as dcc
import dash_bootstrap_components as dbc
import dash_html_components as html
from mongotriggers import MongoTrigger 
import time 
import pandas as pd
import plotly.express as px
import numpy as np
#import plotly.express as px
#import pymongo
from pymongo import MongoClient

from image_processing import detect_fire
from image_processing import calc_location_fire

from glob import glob

# grid size (in cm)
GRID_SIZE = 25
GRID_DIMENSIONS = (150,200)

# Connect to local server
#client = MongoClient("mongodb://127.0.0.1:27017/")

client = MongoClient("mongodb://169.234.54.191:27017/")

# Create database called images
imagedb = client["images"]
# Create Collection (table) called currentImages
imageCollection = imagedb.currentImages

# Create database called waypoints
waypointsdb = client["waypoints"]
# Create Collection (table) called currentWaypoints
waypointsCollection = waypointsdb.currentWaypoints

# Create database called fireMap for our grid
firedb = client["fireMap"]
# Create Collection (table) called grids
gridCollection = firedb.gridStates
EFACollection = firedb.wpEFA



def trigger_image_processing(op_document):
    '''
    triggers image processing for whenever an image is stored
    '''
    #print("trigger_image_processing called")

    num_images = len(glob("./test_images_results/*"))


    fire_list = detect_fire(op_document['o']['path'],
                f'./test_images_results/image_waypoint_{num_images}_results.png')
    
    fire_list = calc_location_fire(op_document['o']['location'], fire_list)

    print()
    print(fire_list)
    print()

    for fire in fire_list:

        # top left
        tLx, tLy = GRID_SIZE * round(fire[0]/GRID_SIZE), GRID_SIZE * round(fire[1]/GRID_SIZE)
        gridCollection.update_one({'location': (tLx,tLy)},{ '$set': {'state': 1.0}})

        # top right
        tRx, tRy = GRID_SIZE * round((fire[0]+fire[2])/GRID_SIZE), GRID_SIZE * round(fire[1]/GRID_SIZE)
        gridCollection.update_one({'location': (tRx,tRy)},{ '$set': {'state': 1.0}})

        # bottom left
        bLx, bLy = GRID_SIZE * round(fire[0]/GRID_SIZE), GRID_SIZE * round((fire[1]+fire[3])/GRID_SIZE)
        gridCollection.update_one({'location': (bLx,bLy)},{ '$set': {'state': 1.0}})

        # bottom right
        bRx, bRy = GRID_SIZE * round((fire[0]+fire[2])/GRID_SIZE), GRID_SIZE * round((fire[1]+fire[3])/GRID_SIZE)
        gridCollection.update_one({'location': (bRx,bRy)},{ '$set': {'state': 1.0}})
        

def create_Grids(gridSize, client, dimensions):
    '''
    Creates a brand new grid given the the length and width
    '''
    gridCollection = client["fireMap"].gridStates
    w = 0
    l = 0
    cx,cy = dimensions[0]/2,dimensions[1]/2
    for i in range(1,int((dimensions[0] * dimensions[1])/(gridSize*gridSize)) + 1):
        if w >= int(dimensions[0]/gridSize):
            w = 0
            l += 1
        gridCollection.insert_one({
            'Grid ID': i, #f"{(int((gridSize*w-cx)/gridSize), int((cy-gridSize*l)/gridSize))}",
            'Location': f"{(gridSize*w-cx, cy-gridSize*l)}",
            'State': 0.0,
            'Time': -1,
            'Cords': (int((gridSize*w-cx)/gridSize)+3, int((cy-gridSize*l)/gridSize)+3)
        })
        w += 1


def create_EFA(gridSize, client, dimensions):
    '''
    Creates a brand new grid given the the length and width
    '''
    EFACollection = client["fireMap"].wpEFA
    w = 0
    l = 0
    cx,cy = dimensions[0]/2,dimensions[1]/2
    for i in range(1,int((dimensions[0] * dimensions[1])/(gridSize*gridSize)) + 1):
        if w >= int(dimensions[0]/gridSize):
            w = 0
            l += 1
        EFACollection.insert_one({
            'Grid ID': i, #f"{(int((gridSize*w-cx)/gridSize), int((cy-gridSize*l)/gridSize))}",
            'EFA': -1,
            'Cords': (int((gridSize*w-cx)/gridSize)+3, int((cy-gridSize*l)/gridSize)+3)
        })
        w += 1


def _create_card(gridNum, grids):
    '''
    Helper function that helps create the cards used in our
    grid map
    '''
    location = grids[gridNum-1]['Location']
    fireState = grids[gridNum-1]['State']
    print(f"check", location, fireState, fireState==1)
    return dbc.Card(
        dbc.CardBody(
            [
                html.P(f'{location}', id=f"Grid-{gridNum}", style={"color":"white", "font-size": "6px"}),
            ],
        ),
        #color='red',
        color = "red" if fireState == 1 else "green",
        style = {"height": f"{round(GRID_SIZE * 3.2)}px", "width": f"{round(GRID_SIZE * 3.2)}px"}
    )


def _create_card_row(row_number, columns, grids):
    '''
    Helper function that helps create a row of cards used in our
    grid map
    '''
    
    return dbc.Row(
        [dbc.Col([_create_card(
            row_number * columns + i + 1, grids)]) for i in range(columns)],
        className="g-0",
        )


def create_grid_map(gridSize, dimensions, grids):
    '''
    Creates a map of grids to be used in our dashboard
    '''
    columns = int(dimensions[0]/gridSize)
    rows = int(dimensions[1]/gridSize)
    #print(f"check columns rows {columns} {rows}")
    components = [_create_card_row(r,columns, grids) for r in range(rows)]
    #print(f"check components {components}")
    return components


# to run the dashboard
if __name__ == "__main__":

    # triggers
    triggers = MongoTrigger(client)
    
    
    #Statetrigger = MongoTrigger(client)
    
    triggers.register_insert_trigger(trigger_image_processing, 'images', 'currentImages')
    
    
    
    triggers.tail_oplog()
    
    #Statetrigger.tail_oplog()
    

    waypointsCollection.delete_many({})

    gridCollection.delete_many({})
    
    EFACollection.delete_many({})

    waypointsCollection.insert_many([{"Round": "0","x": "0", "y": "-25", "z": "110" },
                                    {"Round": "0", "x": "50", "y": "-25", "z": "110" },
                                    {"Round": "0", "x": "50", "y": "25", "z": "110" },
                                    {"Round": "0", "x": "0", "y": "25", "z": "110", },
                                    {"Round": "0", "x": "-50", "y": "25", "z": "110"},
                                    {"Round": "0", "x": "-50", "y": "75", "z": "110"},
                                    {"Round": "0", "x": "0", "y": "75", "z": "110"},
                                    {"Round": "0", "x": "50", "y": "75", "z": "110"},])

    create_Grids(GRID_SIZE, client, GRID_DIMENSIONS)
    
    create_EFA(GRID_SIZE, client, GRID_DIMENSIONS)
    
    
    
    app = JupyterDash(__name__, suppress_callback_exceptions=True,
                    external_stylesheets=['https://codepen.io/chriddyp/pen/bWLwgP.css',dbc.themes.BOOTSTRAP])
    colors = {
    'background': '#F0FBFA',
    'text': '#B41907'}
    app.layout = html.Div(style={'backgroundColor': colors['background']},children=[
        html.H1(children='Sparx Dashboard', style={'textAlign': 'center','color': colors['text'], 'fontSize': 40}),
        html.H4("Image Table", style={'fontSize': 24, 'margin-left': '20px'}),

        # Image datatable
        html.Div(id='image-datatable', children=[],style={'margin-left': '10px'}),

        # Refreshes Images datatable with mongodb data; activated once/week or when page refreshed
        dcc.Interval(id='image_interval_db', interval=86400000 * 7, n_intervals=0),

        html.Div(style={ "height":"25px" }),

        # Waypoints datatable
        html.H4("Waypoints Table",style={'fontSize': 24,'margin-left': '10px'}),

        html.Div(id='waypoints-datatable', children = [], style={'display': 'block', 'margin-left': '20px', 'align-items': 'center', 'justify-content': 'center', 'text-align':'center','fontSize': 20, 'width': '30%'}),

        dcc.Interval(id='waypoints_interval_db', interval=86400000 * 7, n_intervals=0),

        html.Button("Save to Waypoints Database", id="save-it", style={'fontSize': 18, 'border': 'thin black solid',  'margin-left': '10px',
                   'margin-right': '10px'} ),
        html.Button('Add Row', id='adding-rows-btn', n_clicks=0, style={'fontSize': 18, 'border': 'thin black solid','margin-right': '10px'}),

        html.Div(id='placeholder', children = []),
        html.Div(id='placeholder2', children = []),

        html.Div(style={ "height":"25px" }),
        
        html.H4("Grid State", style={'fontSize': 24 , 'margin-left': '10px'}),
        
        # Refreshes FireMap datatable with mongodb data; activated once/week or when page refreshed
        dcc.Interval(id='fire_interval_db', interval=86400000 * 7, n_intervals=0),

        # Grids datatable
        html.Div(children=[
        
        html.Div(id='grid-datatable',  children = [], style={'display': 'inline-block', 'margin-left': '20px','margin-right': '100px', 'align-items': 'center', 'text-align':'center','fontSize': 20, 'width': '50%'}),
        
        # map of grids
        html.Div(id='fire-map-grids', children = [], style={'display': 'inline-block',"align": "center", "width": "480px", "height": "600px",
                "text-align": "center", "margin": "0 auto",'fontSize': 18 }),
        ]),
        
        dcc.Interval(id='fire_map_grids_interval', interval=86400000 * 7, n_intervals=0),
        
        
        html.Button("Save to State Database", id="save-state", style={'fontSize': 18, 'border': 'thin black solid',  'margin-left': '10px',
                   'margin-right': '10px'} ),
        html.Div(style={ "height":"25px" }),

        html.H4("Gird Fire Arrival Time",style={'fontSize': 24,'margin-left': '10px'}),
    
        html.Div(children=[
        html.Div(id='grid-EFA',  children = [], style={'display': 'inline-block', 'margin-left': '20px','margin-right': '100px', 'align-items': 'center', 'text-align':'center','fontSize': 20, 'width': '40%'}),
        dcc.Graph(id='grid-EFA_graph', style={'display': 'inline-block'})#, 'margin-left': '20px'})#,'margin-right': '100px', 'align-items': 'center', 'text-align':'center','fontSize': 20, 'width': '40%'})
        ]),
        dcc.Interval(id='EFA_grids_interval', interval=86400000 * 7, n_intervals=0),
        dcc.Interval(id='EFA_graph_interval', interval=86400000 * 7, n_intervals=0)])


    #f"{round(GRID_DIMENSIONS[1]/float(GRID_DIMENSIONS[0])*700)}
    
    # Display Image Datatable with data from Mongo database *************************
    @app.callback(Output('image-datatable', 'children'),
                [Input('image_interval_db', 'n_intervals')])
    def populate_image_datatable(n_intervals):
        #print(n_intervals)
        # Convert the Collection (table) date to a pandas DataFrame
        df = pd.DataFrame(list(imageCollection.find()))
        #Drop the _id column generated automatically by Mongo
        df = df.iloc[:, 1:]

        return [
            dash_table.DataTable(
                id='image-table',
                columns=[{
                    'name': x,
                    'id': x,
                } for x in df.columns],
                data=df.to_dict('records'),
                editable=True,
                row_deletable=True,
                filter_action="native",
                filter_options={"case": "sensitive"},
                sort_action="native",  # give user capability to sort columns
                sort_mode="single",  # sort across 'multi' or 'single' columns
                page_current=0,  # page number that user is on
                page_size=6,  # number of rows visible per page
                style_cell={'textAlign': 'left', 'minWidth': '100px',
                            'width': '100px', 'maxWidth': '100px'},
            )
        ]
    # Display Grid Datatable with data from Mongo database *************************
    @app.callback(Output('grid-datatable', 'children'),
                [Input('fire_interval_db', 'n_intervals') ])
    def populate_fire_datatable(n_intervals):
        #print(n_intervals)
        # Convert the Collection (table) date to a pandas DataFrame
        df = pd.DataFrame(list(gridCollection.find()))
        #Drop the _id column generated automatically by Mongo
        df = df.iloc[:, 1:]
        Gridindex=['Grid ID', 'Location','State','Time']
        return [
            dash_table.DataTable(
                id='fire-table',
                columns=[{
                    'name': x,
                    'id': x,
                } for x in Gridindex],
                data=df.to_dict('records'),
                editable=True,
                row_deletable=True,
                filter_action="native",
                filter_options={"case": "sensitive"},
                sort_action="native",  # give user capability to sort columns
                sort_mode="single",  # sort across 'multi' or 'single' columns
                page_current=0,  # page number that user is on
                page_size=12,  # number of rows visible per page
                style_cell={'textAlign': 'left', 'minWidth': '100px',
                            'width': '100px', 'maxWidth': '100px'},
            )
        ]
    # Display Waypoints Datatable with data from Mongo database *************************
    @app.callback(Output('waypoints-datatable', 'children'),
                [Input('waypoints_interval_db', 'n_intervals')])
    def populate_WP_datatable(n_intervals):
        #print(n_intervals)
        # Convert the Collection (table) date to a pandas DataFrame
        df = pd.DataFrame(list(waypointsCollection.find()))
        #Drop the _id column generated automatically by Mongo
        df = df.iloc[:, 1:]
        return [
            dash_table.DataTable(
                id='waypoints-table',
                columns=[{
                    'name': x,
                    'id': x,
                } for x in  df.columns],
                data=df.to_dict('records'),
                editable=True,
                row_deletable=True,
                filter_action="native",
                filter_options={"case": "sensitive"},
                sort_action="native",  # give user capability to sort columns
                sort_mode="single",  # sort across 'multi' or 'single' columns
                page_current=0,  # page number that user is on
                page_size=6,  # number of rows visible per page
                style_cell={'textAlign': 'left', 'minWidth': '100px',
                            'width': '100px', 'maxWidth': '100px'},
            )
        ]
    

    # Display Grid EFA with data from Mongo database *************************
    @app.callback(Output('grid-EFA', 'children'),
                [Input('EFA_grids_interval', 'n_intervals')])
    def populate_EFA_datatable(n_intervals):
        # Convert the Collection (table) date to a pandas DataFrame
        df = pd.DataFrame(list(EFACollection.find()))
        #Drop the _id column generated automatically by Mongo
        df = df.iloc[:, 1:]
        Gridindex=['Grid ID', 'EFA']
        return [
            dash_table.DataTable(
                id='EFA-table',
                columns=[{
                    'name': x,
                    'id': x,
                } for x in Gridindex],
                data=df.to_dict('records'),
                editable=True,
                row_deletable=True,
                filter_action="native",
                filter_options={"case": "sensitive"},
                sort_action="native",  # give user capability to sort columns
                sort_mode="single",  # sort across 'multi' or 'single' columns
                page_current=0,  # page number that user is on
                page_size=12,  # number of rows visible per page
                style_cell={'textAlign': 'left', 'minWidth': '100px',
                            'width': '100px', 'maxWidth': '100px'},
            )
        ]
    
     # Display Grid EFA Graph with data from Mongo database *************************
    @app.callback(Output('grid-EFA_graph', 'figure'),
                [Input('EFA_graph_interval', 'n_intervals')])
    def populate_EFA_Graph(n_intervals):
        # Convert the Collection (table) date to a pandas DataFrame
        df = pd.DataFrame(list(EFACollection.find()))
        #Drop the _id column generated automatically by Mongo
        df = df.iloc[:, 1:]
        Gridindex=['Grid ID', 'EFA']
        
        cord=list(df['Cords']) 
        efa=list(df['EFA'])
        shape=np.array(GRID_DIMENSIONS)//GRID_SIZE
        Grid_EFA=np.full((6,8),0)
        for i in range(len(cord)):
            x,y=cord[i]
            Grid_EFA[x,y]=efa[i]
        print(f"see why {Grid_EFA}")
        fig=px.imshow(Grid_EFA, origin='lower')
        
        return fig
    
    
    
    
    
    
    
    # Display map of Fire Grids with data from Mongo database *************************
    @app.callback(Output('fire-map-grids', 'children'),
                [Input('fire_map_grids_interval', 'n_intervals')])#,Input("save-state", "n_clicks")])
    def Draw_Grid_Map(n_intervals):#,n_clicks
        #print(n_intervals)
        while True:
            df = list(gridCollection.find())
            if len(df)==48: 
               # print(f"check the length {len(df)}")
                break
            else:
                time.sleep(1)
            
        return create_grid_map(GRID_SIZE, GRID_DIMENSIONS, df)



    # Add new rows to Waypoints DataTable ***********************************************
    @app.callback(
        Output('waypoints-table', 'data'),
        [Input('adding-rows-btn', 'n_clicks')],
        [State('waypoints-table', 'data'),
        State('waypoints-table', 'columns')],
    )
    def add_row(n_clicks, rows, columns):
        print("TESTING 1")
        if n_clicks > 0:
            rows.append({c['id']: '' for c in columns})
        return rows

    # Save new Waypoints DataTable data to the Mongo database ***************************
    @app.callback(
        Output("placeholder", "children"),
        Input("save-it", "n_clicks"),
        State("waypoints-table", "data"),
        prevent_initial_call=True
    )
    def save_data(n_clicks, data):
        print("TESTING 2")
        dff = pd.DataFrame(data)
        waypointsCollection.delete_many({})
        waypointsCollection.insert_many(dff.to_dict('records'))
        return ""
    
    
    @app.callback(
        Output("placeholder2", "children"),
        Input("save-state", "n_clicks"),
        State("fire-table", "data"),
        prevent_initial_call=True
    )
    def save_state(n_clicks, data):
        print("TESTING 2")
        dff = pd.DataFrame(data)
        gridCollection.delete_many({})
        gridCollection.insert_many(dff.to_dict('records'))
        return ""


    app.run_server(mode='jupyterlab')
    triggers.stop_tail()





