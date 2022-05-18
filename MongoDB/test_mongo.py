import plotly.express as px
import pandas as pd
import pymongo
from pymongo import MongoClient

# Connect to local server
client = MongoClient("mongodb://127.0.0.1:27017/")

# Create database called animals
mydb = client["Command"]
# Create Collection (table) called shelterA
collection = mydb.firefighter

record = {
    "Area": 1,
    "Time": 0,
    "Event": "Human",
    "Frequency (TPM)": 1,
    "Sign.": 3,
}
#collection.insert_one(record)
mydb=client['Fire_setting']
collection = mydb.status

record = {
    "Area": 2,
    "Fire": 'Yes',
    "Fire Time": 1,
    "Window": 'close',
    "Window Time": 3,
    "Human": 'Yes',
    "Human Time": -1,
}

collection.insert_one(record)


# testing = collection.find_one()
# print(testing)

# Convert the Collection (table) date to a pandas DataFrame
# df = pd.DataFrame(list(collection.find()))
# print(df)
# print("----------------------------")
# print(df.iloc[:, 1:])
