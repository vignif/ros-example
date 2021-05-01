# Database handler

This node creates at runtime an sqlite database and a table `cities` with a defined schema.
It represents the direct access to the database object.
Clients can use the method `CreateCity` in order to store a new row in the db.
In the root of this package the database file `test.db` will be created at runtime.
Implemented as a library.