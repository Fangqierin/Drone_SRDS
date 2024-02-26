import dash     # need Dash version 1.21.0 or higher
from dash.dependencies import Input, Output, State
import dash_table
import pandas as pd
import dash_core_components as dcc
import dash_html_components as html
import dash_bootstrap_components as dbc
import dash_bio as dashbio
from dash_bio.utils import PdbParser, create_mol3d_style
from dash.dependencies import Input, Output
import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt
import mpld3 
import pymongo
from pymongo import MongoClient
from bson import ObjectId

# Connect to local server
client = MongoClient("mongodb://127.0.0.1:27017/")
# Create database called animals
mydb = client["Command"]
# Create Collection (table) called shelterA
collection = mydb.wins
windows=pd.read_csv('../data/all_win.csv', delim_whitespace=True)
#print(windows)
collection.delete_many({})
for index, row in windows.iterrows():
        #print(windows.columns)
#         print(row._get_value('id'))
#         print(row['id'])
        record={"id":row["id"], "x":row["v_1_x"], "y":row["v_1_y"], 
              "z":row["v_1_z"], 'win': 'Close', 'fire': 'None', 'hum': 'None'}
        collection.insert_one(record)
# #Read Fire Simulation 
# myquery = { "id": 0 }
# w=collection.find(myquery)[0]
# print(w)
# w['fire']='Burn'
# collection.update_one(myquery,w)     
#testing = collection.find_one()

