# Geo Manager

Main node of the project, it transmit the requests to the `api_handler` and expects a proper response.
A unique pointer to the database manages the access to it and is used also for storing new cities.
Geo manager also publishes the topic used for adding new cities at runtime `/RTCreateCity`.
