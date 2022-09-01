# Chicago Public School Data Analysis using SQL & Python

This project involves analysing and answering performance based questions pertaining to Chicago public schools dataset with the aid of Python libraries (Pandas, Matplotlib, Seaborn) and Jupyter SQL magic functions. The dataset, stored in an Db2 database on IBM Cloud instance, contains engagement metrics (student/teacher attendance), locations (city, state, zip code), safety score, family involvement score, college elligbility, college enrollment rate, graduation rate, etc.

## Description

An in-depth overview of techniques and methods employed in this project are as follows:
- **Data Loading:** Tasks include manually loading the table using the database console LOAD tool (as opposed to reading the dataset into a Pandas dataframe and then PERSIST it into the database to achieve optimal SQL querying), establsihing a connection with the database (**%sql ibm_db_sa://my-username:my-password@my-hostname:my-port/my-db-name?security=SSL**) and querying the database system catalog to retrieve table metadata.
- **Data Exploration:** Some of the functions include **CAST()** to change column datatype to conform to numeric comparison in where clauses, **REPLACE()** for improved readibility of output table, using **Group By, MIN, MAX, AVG, SUM & COUNT** for aggregate analysis and plotting bar charts & scatter plots to visualise data.

## Getting Started

### Dependencies

* Jupyter notebook required. Details can be found at https://jupyter.readthedocs.io/en/latest/install.html 

### Installing

* Save the files from the folder named Datasets (click the green "clone or download" button and then click "Download ZIP").
* Read the files by using Pandas's read_csv method(**pd.read_csv("filepath")**).

### Executing program

* Run the cells in Jupyter notebook by Shift + Enter (Mac) or pressing run button in the top navigation bar.

## Help

In case of any issues with Jupyter notebook, refer to this link https://jupyter-notebook.readthedocs.io/en/stable/troubleshooting.html

## Authors

Muhammad Ali
[@ma-94](https://www.linkedin.com/in/muhammadali7/)

## License

This project is licensed under the MIT License - see the LICENSE.md file in the parent repository (Data-Science-Portfolio) for details.
