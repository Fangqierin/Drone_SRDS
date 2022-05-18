import dash     # need Dash version 1.21.0 or higher
from dash.dependencies import Input, Output, State
import dash_table
import dash_core_components as dcc
import dash_html_components as html

import pandas as pd
import plotly.express as px
import pymongo
from pymongo import MongoClient
from bson import ObjectId

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
#collection.insert_one(record)
#testing = collection.find_one()
#print(testing)

collection_fire=client['Fire_setting'].status

colors = {
    'background': '#F0FBFA',
    'text': '#B41907'
}

app = dash.Dash(__name__, suppress_callback_exceptions=True,
                external_stylesheets=['https://codepen.io/chriddyp/pen/bWLwgP.css'])

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
    html.Div(children='''Task statistic.'''),
    html.Div(id="show-graphs", children=[]),
    html.Div(id="drone-graphs", children="Status of Drones"),
    html.Div(id="placeholder")

])
# Display Datatable with data from Mongo database *************************
@app.callback(Output('mongo-datatable', 'children'),
              [Input('interval_db', 'n_intervals')])
def populate_datatable(n_intervals):
    #print(n_intervals)
    # Convert the Collection (table) date to a pandas DataFrame
    df = pd.DataFrame(list(collection.find()))
    #Drop the _id column generated automatically by Mongo
    df = df.iloc[:, 1:]
    #print(df.head(6))
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
            style_data={ 'whiteSpace': 'normal', 'height': 'auto', 'lineHeight': '15px'},)
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


# Create graphs from DataTable data ***************************************
@app.callback(
    Output('show-graphs', 'children'),
    Input('my-table', 'data')
)
def add_row_2(data):
    df_grpah = pd.DataFrame(data)
    fig_hist1 = px.histogram(df_grpah, x='Event')
    fig_hist2 = px.histogram(df_grpah, x="Frequency (TPM)")
    return [
        html.Div(children=[dcc.Graph(figure=fig_hist1)], className="three columns"),
        html.Div(children=[dcc.Graph(figure=fig_hist2)], className="three columns")
    ]


if __name__ == '__main__':
    app.run_server(debug=True)
