## Wind_IOT
Este proyecto propone un método para detectar la velocidad del viento a partir de datos provenientes de un sensor de 9 ejes (acelerómetro, giróscopo, magnetómetro). El dispositivo se coloca en un elemento que gire en función de la velocidad del viento, por ejemplo un anemómetro o un aerogenerador eólico. 

En una primera fase se recogen los datos obtenidos por un sensor inercial de 9 ejes junto con la medida real de la velocidad del viento, posteriormente se realiza la calibración del dispositivo.

A continuación, se desarrolla la algoritmia, se obtienen relaciones polinómicas y lineales entre los datos del sensor y la velocidad del viento. La calibración se realiza observando la correlación entre los datos del sensor y la velocidad real medida con un anemómetro.  

En la última fase el dispositivo alimentado por baterías es colocado en las superficies anteriormente mencionadas comportandose como un dispositivo Bluetooth de sensado del medioambiente el cual puede acceder cualquier dispositivo que tenga Bluetooth BLE, el dispositivo envia una característica ble con posibilidad de notificar con la velocidad real del viento True Wind Speed.  

# Codigo para la captura de datos.
El dispositivo que se encuentra en una superficie rotativa utiliza este código para capturar los datos del acelerómetro, giróscopo y magnetómetro (en cada uno de sus ejes x, y, z) los datos son enviados mediante características ble, para no perder precisión antes del envió son convertidos de coma flotante a hexadecimal, se envía un total de 49 caracteres cada segundo.

El dispositivo que captura la velocidad real mediante otro sensor (e.g. anemómetro) se encuentra estático y es el encargado de recibir la información inercial mediante las características bluetooth del primer dispositivo, utiliza este código para recibir, recomponer y enviar los datos por puerto serie para su análisis.

# Codigo para la estimación de la velocidad del viento.
Este código se utiliza para la aplicación final, donde el dispositivo ya calibrado estima la velocidad del viento, el dispositivo funciona como un sensor ambiental comercial, envía cada segundo mediante notificaciones BLE los datos de la velocidad del viento que pueden ser consultados por cualquier usuario que se encuentre dentro del rango del dispositivo.
