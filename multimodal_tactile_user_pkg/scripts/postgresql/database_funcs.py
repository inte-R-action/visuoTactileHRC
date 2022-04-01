# Adapted from https://www.postgresqltutorial.com/postgresql-python/connect/
#!/usr/bin/env python3.7
import psycopg2
import os
from configparser import ConfigParser
import re
import ntpath
import sys

class database():
    def __init__(self):
        self.conn = None
        self.cur = None
        self.db_params = None
        # read connection parameters
        self.config()

    def config(self, filename=os.path.dirname(__file__)+'/database.ini', section='postgresql'):
        # create a parser
        parser = ConfigParser()
        # read config file
        parser.read(filename)
        # get section, default to postgresql
        self.db_params = {}
        if parser.has_section(section):
            params = parser.items(section)
            for param in params:
                self.db_params[param[0]] = param[1]
        else:
            raise Exception('Section {0} not found in the {1} file'.format(section, filename))

    def connect(self, verbose=False):
        """ Connect to the PostgreSQL database server """
        self.conn = None
        try:
            # connect to the PostgreSQL server
            if verbose:
                print('Connecting to the PostgreSQL database...')
            self.conn = psycopg2.connect(**self.db_params)
            # create a cursor
            self.cur = self.conn.cursor()
            
            # execute a statement
            self.cur.execute('SELECT version()')
            # display the PostgreSQL database server version
            db_version = self.cur.fetchone()
            if verbose:
                print('Connected! PostgreSQL database version:')    
                print(db_version)
        except (Exception, psycopg2.DatabaseError) as error:
            print(f"db connect error: {error}")
            if self.conn is not None:
                self.conn.close()
                print('Database connection closed.')
            raise
        
    def disconnect(self, verbose=False):
        try:
        # close the communication with the PostgreSQL
            self.cur.close()
        except AttributeError as e:
            print(f"Close database error: {e}") 
        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            raise
        finally:
            if self.conn is not None:
                self.conn.close()
                if verbose:
                    print('Database connection closed.')

    def insert_data_list(self, table, columns, data, verbose=True):
        """ insert multiple data rows into table  
        table:str(table name), columns:[str(column name),], data:[('data1', data2, ),]"""
        data_ins = "%s" + (", %s"*(len(columns)-1))
        separator = ', '
        sql = f"INSERT INTO {table}({separator.join(columns)}) VALUES({data_ins})"
        try_again = True
        while try_again:
            self.connect()
            try:
                # execute the INSERT statement
                self.cur.executemany(sql, data)
                # commit the changes to the database
                self.conn.commit()
                if verbose:
                    print(f"Data successfully inserted into {table} table")
                self.disconnect()
                break
            except (Exception, psycopg2.DatabaseError) as error:
                print(f"Insert db error: {error}")
                print(f"Try again: {try_again}")
                try_again = False
                self.disconnect()
                if not try_again:
                    raise
    
    def csv_export(self, table, file_path=None, verbose=True):
        # https://kb.objectrocket.com/postgresql/from-postgres-to-csv-with-python-910
        """ Export table to csv file
        table:str(table name), 
        filepath:str(full filepath incl. filename.csv, default=../scripts/postgresql/{table}.csv)"""

        self.connect()

        # Use the COPY function on the SQL we created above.
        sql = f"COPY (SELECT * FROM {table}) TO STDOUT WITH CSV HEADER"

        # Set up a variable to store our file path and name.
        if file_path is None:
            file_path = os.path.dirname(__file__)+f'/{table}.csv'

        # Trap errors for opening the file
        try:
            with open(file_path, 'w') as f_output:
                self.cur.copy_expert(sql, f_output)
            print(f"Table {table} successfully exported to {file_path}")
        except psycopg2.Error as e:
            print(f"Error: {e}/n query we ran: {sql}/n file_path: {file_path}")
            raise
        except FileNotFoundError as e:
            if verbose:
                print(f"Export csv error: {e}")
            raise
        except Exception as e:
            if verbose:
                print(f"Export csv error: {e}")
            raise
        finally:
            self.disconnect()
            
    def create_table(self, name, columns):
        """Create table in the PostgreSQL database
        name:str(table name), columns:[str(table command)]
        Table command examples:
        vendor_id SERIAL PRIMARY KEY,
        vendor_name VARCHAR(255) NOT NULL,
        drawing_data BYTEA NOT NULL,
        FOREIGN KEY (part_id) REFERENCES parts (part_id) ON UPDATE CASCADE ON DELETE CASCADE"""

        self.connect()
        separator = ', '
        sql = f"CREATE TABLE {name} ({separator.join(columns)})"

        try:
            # create table one by one
            self.cur.execute(sql)
            # commit the changes
            self.conn.commit()
            print(f"Successfully created {name} table")
        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            raise
        finally:
            self.disconnect()

    def csv_import(self, filepath, header=True, tab_name=None):
        """Import csv file into table
        filepath:str(full filepath), header:bool (default True) if csv includes header
        tab_name: name of table to insert into, default name of file specified"""

        # Trap errors for opening the file
        filename = ntpath.basename(filepath)
        if tab_name is None:
            try:
                tab_name = re.search('(.*).csv', filename)
            except Exception as e:
                print(f"Table name construction error: {e}")
                raise

        try:
            f_contents = open(filepath, 'r')
        except psycopg2.Error as e:
            print(f"Database error: {e} open() text file: {filepath}")
            raise

        if header:
            sql = f"COPY {tab_name} FROM stdin WITH CSV HEADER DELIMITER as ','"
        else:
            sql = f"COPY {tab_name} FROM stdin WITH CSV DELIMITER as ','"

        # Trap errors for copying the array to our database
        try:
            self.connect()
            self.cur.copy_expert(sql=sql, file=f_contents)
            self.conn.commit()
            print(f"Successfully inserted data from {filename} into {tab_name} table")
        except psycopg2.Error as e:
            print(f"Database error: {e} copy_expert")
            raise
        finally:
            self.disconnect()

    def gen_cmd(self, command):
        """Generic postgrsql commmand"""

        self.connect()

        try:
            # create table one by one
            self.cur.execute(command)
            # commit the changes
            self.conn.commit()
        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            raise
        finally:
            self.disconnect()

    def table_list(self, t_schema = "public", verbose=True):
        # Retrieve the table list
        sql = "SELECT table_name FROM information_schema.tables WHERE ( table_schema = '" + t_schema + "' ) ORDER BY table_name;"
        list_tables = None
        try:
            self.connect()
            # Retrieve all the rows from the cursor
            self.cur.execute(sql)
            list_tables = [i for t in self.cur.fetchall() for i in t]
            
            # Print the names of the tables
            if verbose:
                print(f"Tables currently available: {list_tables}")
        except psycopg2.Error as e:
            print(f"Database error: {e} copy_expert")
            raise
        finally:
            self.disconnect()

        return list_tables

    def remove_table(self, name):
        """Remove table from database
        name=str(table name)"""

        self.connect()
        sql = f"DROP TABLE {name} CASCADE;"
        try:
            # create table one by one
            self.cur.execute(sql)
            # commit the changes
            self.conn.commit()
            print(f"Successfully removed table {name}")
        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            raise
        finally:
            self.disconnect()

    def query_table(self, name, rows='all'):
        """Query rows from table
        name=str(table name), rows='all', 'one', int"""

        sql = f"SELECT * FROM {name}"
        output = None
        try:
            self.connect()
            self.cur.execute(sql)
            if rows == 'all':
                output = self.cur.fetchall()
            elif rows =='one':
                output = self.cur.fetchone() 
            elif type(rows) == int:
                output = self.cur.fetchmany(rows)
            else:
                raise TypeError('query_table rows input should be \'all\', \'one\' or int')

            col_names = []
            for i in self.cur.description:
                col_names.append(i[0])

        except (Exception, psycopg2.DatabaseError) as error:
            print(error)
            raise
        finally:
            self.disconnect()

        return col_names, output

            



if __name__ == '__main__':
    db = database()
    #db.connect()
    #db.create_table('test1', ["vendor_id SERIAL PRIMARY KEY", "vendor_name VARCHAR(255) NOT NULL"])
    # db.insert_data_list("parts", ["part_name"], [
    #     ('AKM Semiconductor Inc.',),
    #     ('Asahi Glass Co Ltd.',),
    #     ('Daikin Industries Ltd.',),
    #     ('Dynacast International Inc.',),
    #     ('Foster Electric Co. Ltd.',),
    #     ('Murata Manufacturing Co. Ltd.',)
    # ])
    # db.insert_data_list("parts", ["part_name", "something"], [
    #     ('.txt','\\x1234',),
    #     ('.jpg','\\x1234',),
    #     ('.jpg','\\x1234',),
    #     ('.jpg','zdfb',)
    # ])
    #db.csv_import(os.getcwd()+'/sam_nodes/scripts/postgresql/users.csv', tab_name="vendors")
    #db.csv_export("parts")
    print(db.query_table("episodes", 'all'))
    #db.disconnect()
    #db.table_list()
    #db.remove_table('test1')