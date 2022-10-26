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
client = MongoClient("mongodb://140.114.89.210:27017/")
# Create database called animals
mydb = client["Command"]
# Create Collection (table) called shelterA
collection = mydb.firefighter


if __name__ == '__main__':
    floor_num=12
    va=3
    client = MongoClient("mongodb://140.114.89.210:27017/")
    mydb = client["Command"]
    #print(mydb.list_collection_names())
    # Create Collection (table) called shelterA
    collection = mydb.WPS
    df = pd.DataFrame(list(collection.find()))
    print(df)

