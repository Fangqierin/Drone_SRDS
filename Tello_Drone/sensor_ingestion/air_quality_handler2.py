import quantaq
import requests

## API key of the quant AQ sensor in lab
api_key = "IP0N56R2VKAK1Z44XR1CZYPD"
## Serial Number of the quant AQ sensor in lab
sn = "MOD-PM-00174"
## URL of enrichDB
url = 'http://128.195.52.200:5000/observation/air_quality'

class qt_aq_sensor():
    """Quant AQ sensor class"""
    def __init__(self,api_key=None):
        """ Set the API client by the provided api_key """
        self.__client = quantaq.QuantAQAPIClient(api_key=api_key)
    def get_client(self):
        """Get the client object"""
        return self.__client
    def get_data_from_dates(self,sn,start_date,end_date):
        """
        Get the data from a start date to a end date
        from a device with serial number (sn)
        by a API of QuantAQ
        start_date: string (yyyy-mm-dd)
        end_date: string (yyyy-mm-dd)
        sn: string (defined by QuantAQ)
        """
        return self.get_client().data.list(sn=sn, start=start_date, stop=end_date)


class data_handler():
    """Data handler class"""
    def __init__(self,url):
        """Set the url of the server (EnrichDB)"""
        self.__url = url
    def get_url(self):
        """Get the url"""
        return self.__url
    def post_data(self,data):
        """
        Post the data to the server
        data: list of dicts
        """
        for d in data:
            ## Cast data into the format accepted by the server
            upload_d = {}
            upload_d["arrival_time"] = d["timestamp_local"]
            gps = d["geo"]
            gps_string = str(gps["lat"])+","+str(gps["lon"])
            upload_d["gps"] = gps_string
            upload_d["pm1"] = d["pm1"]
            upload_d["pm10"] = d["pm10"]
            upload_d["pm25"] = d["pm25"]
            met = d["met"]
            met_string = "{\"pressure\": "+str(met["pressure"])+\
            ", \"rh\": "+str(met["rh"])+", \"temp\": "+str(met["temp"])+"}"
            upload_d["metrics"] = met_string
            print(upload_d)
            r = requests.post(self.get_url(), json = upload_d)
            print(r.json())
            if r.json()["result"] != "success":
                print("Post failed!!")
                return
        print("All data are posted successfully!!")

def get_input_date():
    """
    Get the start date and end date
    return a tuple (start_date,end_date)
    """
    start_date = input("Please enter the start date (yyyy-mm-dd):")
    end_date = input("Please enter the end date (yyyy-mm-dd):")
    return (start_date,end_date)

if __name__ == '__main__':

    ## Get the start date and end date of data
    input_date = get_input_date()

    ## Initialize sensor class and data handler class
    qt_aq = qt_aq_sensor(api_key)
    data_handler = data_handler(url)

    ## Get the data from the specified period
    data = qt_aq.get_data_from_dates(sn,input_date[0],input_date[1])
    ## Post the data to the server
    data_handler.post_data(data)
