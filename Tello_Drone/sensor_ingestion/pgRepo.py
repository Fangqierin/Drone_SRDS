from datetime import datetime
import psycopg2
import string


class PostgresRepo:
    def __init__(self):
        self.connection = psycopg2.connect(
            host="localhost",
            port=5432,
            database="test",
            user="ingestor",
            password="")  # TODO: Check password

    def fetch_entity(self, sql: string):
        cursor = self.connection.cursor()
        cursor.execute(sql)
        result = cursor.fetchone()
        count = cursor.rowcount
        cursor.close()
        return result, count

    def fetch_entities(self, sql: string):
        cursor = self.connection.cursor()
        cursor.execute(sql)
        result = cursor.fetchall()
        cursor.close()
        return result

    def execute(self, command: string) -> int:
        cursor = self.connection.cursor()
        cursor.execute(command)
        row_count = cursor.rowcount
        self.connection.commit()
        cursor.close()
        return row_count

    def test_response(self):
        return self.fetch_entities("select * from firedata limit 10")

    def insert_observation(self, table: str, params: dict):
        # keys = [f"'{k}'" for k in params.keys()]
        values = [f"\"{v}\"" for v in params.values()]
        self.execute(f"INSERT INTO {table} "
                     f"({', '.join(params.keys())}) "
                     f"VALUES ({', '.join(values)})")

