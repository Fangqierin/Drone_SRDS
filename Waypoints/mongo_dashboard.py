
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

import pandas as pd
#import plotly.express as px
#import pymongo
from pymongo import MongoClient

from image_processing import detect_fire
from image_processing import calc_location_fire

from glob import glob

# grid size (in cm)
GRID_SIZE = 12.5
GRID_DIMENSIONS = (200,150)

# Connect to local server
client = MongoClient("mongodb://127.0.0.1:27017/")

# Create database called images
imagedb = client["images"]
# Create Collection (table) called currentImages
imageCollection = imagedb.currentImages


# Create database called fireMap
firedb = client["fireMap"]
# Create Collection (table) called grids
gridCollection = firedb.gridStates


def trigger_image_processing(op_document):
    '''
    triggers image processing for whenever an image is stored
    '''
    print("trigger_image_processing called")


    fire_list = detect_fire(op_document['o']['path'],
                'test_images_results/image_waypoint_0_results.png')
    
    fire_list = calc_location_fire(op_document['o']['location'], fire_list)

    print(fire_list)

    for fire in fire_list:

        # top left
        tLx, tLy = GRID_SIZE * round(fire[0]/GRID_SIZE), GRID_SIZE * round(fire[1]/GRID_SIZE)
        gridCollection.update_one({'location': (tLx,tLy)},{ '$set': {'fireState': 1.0}})

        # top right
        tRx, tRy = GRID_SIZE * round((fire[0]+fire[2])/GRID_SIZE), GRID_SIZE * round(fire[1]/GRID_SIZE)
        gridCollection.update_one({'location': (tRx,tRy)},{ '$set': {'fireState': 1.0}})

        # bottom left
        bLx, bLy = GRID_SIZE * round(fire[0]/GRID_SIZE), GRID_SIZE * round((fire[1]+fire[3])/GRID_SIZE)
        gridCollection.update_one({'location': (bLx,bLy)},{ '$set': {'fireState': 1.0}})

        # bottom right
        bRx, bRy = GRID_SIZE * round((fire[0]+fire[2])/GRID_SIZE), GRID_SIZE * round((fire[1]+fire[3])/GRID_SIZE)
        gridCollection.update_one({'location': (bRx,bRy)},{ '$set': {'fireState': 1.0}})
        



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
            'gridNum': i,
            'location': (gridSize*w-cx, cy-gridSize*l),
            'fireState': 0.0,
            'gridSize': gridSize
        })
        w += 1



def _create_card(gridNum, grids):
    '''
    Helper function that helps create the cards used in our
    grid map
    '''
    location = grids[gridNum-1]['location']
    fireState = grids[gridNum-1]['fireState']

    print(location, fireState)

    return dbc.Card(
        dbc.CardBody(
            [
                #html.P(f'Grid {gridNum} {location}', id=f"Grid-{gridNum}", style={"color":"white", "font-size": "9px"}),
                html.P(fireState, id=f"Grid-{gridNum}-state", style={"color":"white"})
            ],
        ),
        color = "danger" if fireState == 1 else "success",
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

    components = [_create_card_row(r,columns, grids) for r in range(rows)]

    return components


# to run the dashboard
if __name__ == "__main__":

    # triggers
    triggers = MongoTrigger(client)

    triggers.register_insert_trigger(trigger_image_processing, 'images', 'currentImages')

    triggers.tail_oplog()


    gridCollection.delete_many({})

    create_Grids(GRID_SIZE, client, GRID_DIMENSIONS)



    app = JupyterDash(__name__, suppress_callback_exceptions=True,
                    external_stylesheets=['https://codepen.io/chriddyp/pen/bWLwgP.css',dbc.themes.BOOTSTRAP])

    app.layout = html.Div([

        html.H4("Image Database"),

        # Image datatable
        html.Div(id='image-datatable', children=[]),

        # Refreshes Images datatable with mongodb data; activated once/week or when page refreshed
        dcc.Interval(id='image_interval_db', interval=86400000 * 7, n_intervals=0),

        html.Button("Save to Image Database", id="save-it"),
        html.Button('Add Row', id='adding-rows-btn', n_clicks=0),

        html.Div(style={ "height":"25px" }),

        html.H4("Grid Database"),

        # Grids datatable
        html.Div(id='grid-datatable', children = []),

        # Refreshes FireMap datatable with mongodb data; activated once/week or when page refreshed
        dcc.Interval(id='fire_interval_db', interval=86400000 * 7, n_intervals=0),

        html.Div(style={ "height":"25px" }),

        # Creates a map representation for our grid states
        #create_grid_map(GRID_SIZE, GRID_DIMENSIONS)

        # map of grids
        html.Div(id='fire-map-grids', children = [], 
            style={"align": "center", "height": f"{round(GRID_DIMENSIONS[1]/GRID_DIMENSIONS[0]*700)}px", 
                "width": f"{round(GRID_DIMENSIONS[0]/GRID_DIMENSIONS[1]* 700)}px",
                "text-align": "center", "margin": "0 auto"}),

        dcc.Interval(id='fire_map_grids_interval', interval=86400000 * 7, n_intervals=0)

    ])

    # triggers.register_update_trigger(trigger_update_fire_map, 'fireMap', 'gridStates')

    # Display Image Datatable with data from Mongo database *************************
    @app.callback(Output('image-datatable', 'children'),
                [Input('image_interval_db', 'n_intervals')])
    def populate_image_datatable(n_intervals):
        print(n_intervals)
        # Convert the Collection (table) date to a pandas DataFrame
        df = pd.DataFrame(list(imageCollection.find()))
        #Drop the _id column generated automatically by Mongo
        df = df.iloc[:, 1:]
        print(df.head(20))

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
    
    
    # Display Fire Datatable with data from Mongo database *************************
    @app.callback(Output('grid-datatable', 'children'),
                [Input('fire_interval_db', 'n_intervals')])
    def populate_fire_datatable(n_intervals):
        print(n_intervals)
        # Convert the Collection (table) date to a pandas DataFrame
        df = pd.DataFrame(list(gridCollection.find()))
        #Drop the _id column generated automatically by Mongo
        df = df.iloc[:, 1:]
        print(df.head(20))

        return [
            dash_table.DataTable(
                id='fire-table',
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
    


    # Display map of Fire Grids with data from Mongo database *************************
    @app.callback(Output('fire-map-grids', 'children'),
                [Input('fire_map_grids_interval', 'n_intervals')])
    def populate_fire_datatable(n_intervals):
        print(n_intervals)
        df = list(gridCollection.find())

        return create_grid_map(GRID_SIZE, GRID_DIMENSIONS, df)


    # Add new rows to Image DataTable ***********************************************
    @app.callback(
        Output('image-table', 'data'),
        [Input('adding-rows-btn', 'n_clicks')],
        [State('image-table', 'data'),
        State('image-table', 'columns')],
    )
    def add_row(n_clicks, rows, columns):
        if n_clicks > 0:
            rows.append({c['id']: '' for c in columns})
        return rows


    # Save new Image DataTable data to the Mongo database ***************************
    @app.callback(
        Output("placeholder", "children"),
        Input("save-it", "n_clicks"),
        State("image-table", "data"),
        prevent_initial_call=True
    )
    def save_data(n_clicks, data):
        dff = pd.DataFrame(data)
        imageCollection.delete_many({})
        imageCollection.insert_many(dff.to_dict('records'))
        return ""


    app.run_server(mode='jupyterlab')

    triggers.stop_tail()
    
