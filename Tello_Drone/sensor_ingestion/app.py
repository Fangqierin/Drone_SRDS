from flask import Flask, request
from pgRepo import PostgresRepo


def adapt_air_quality(observation: dict) -> dict:
    return observation


def adapt_image(observation: dict) -> dict:
    return observation


app = Flask(__name__)
repo = PostgresRepo()
adapter_map = {
    'air_quality': adapt_air_quality,
    'image': adapt_image
}

table_map = {
    'air_quality': 'ingest_test',
    'image': 'ingest_test'
}


@app.route('/')
def hello_world():
    return 'Hello World!'


@app.route('/testdb')
def test_response():
    return {'result': repo.test_response()}


@app.route('/observation/<sensor_type>', methods=['GET', 'POST'])
def insert_observation(sensor_type: str):
    params = request.get_json(force=True)
    repo.insert_observation(table_map[sensor_type], adapter_map[sensor_type](params))
    return {'result': 'success'}


if __name__ == '__main__':
    app.run()
