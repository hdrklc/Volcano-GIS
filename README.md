# Project Documentation for Volcano GIS

**🌋 A web-based GIS application for monitoring and analyzing volcanic activity in Europe.**

## 🗺️ Menu

- [**System Architecture**](#system-architecture)
- [**Package Dependency**](#dependency)
- [**Package Installation**](#install)
- [**Prepare Data**](#prepare-data)
- [**Application Features**](#application-features)
- [**Running the Application**](#run-the-application)
- [**Other Notes**](#other-notes)
- [**Issues**](#issues)
- [**Acknowledgements**](#acknowledgements)

## 🛠️ System Architecture

![System Architecture](./docs/system_architecture.png)

The system comprises the following components:

- 🖥️ **Frontend Module:** Interactive map interface for volcano selection and analysis using OpenLayers.
- ⚙️ **Backend Module:** Node.js server managing database interactions and analysis computations.
- 🗄️ **Database Module:** PostgreSQL with PostGIS extension for spatial data management.
- 🤖 **Chatbot Integration:** Provides real-time information about volcanic activity and predictive insights.

This architecture ensures efficient, real-time performance while maintaining accuracy.

## 🔧 Package Dependency

Install the required dependencies with:

```bash
sudo apt-get update
sudo apt-get install -y nodejs npm postgresql postgis
```

Install Node.js dependencies:

```bash
npm install express pg node-fetch openlayers chatbot-api
```

**Dependency Details:**
- 🛠️ **Node.js:** JavaScript runtime environment.
- 🚀 **Express:** Lightweight web server framework.
- 🌐 **OpenLayers:** Library for interactive map rendering.
- 🗺️ **PostGIS:** Spatial database extension for PostgreSQL.
- 🤖 **Chatbot API:** For user interactions.

## 🚀 Package Installation

Follow these steps to install the application:

```bash
cd ~/volcano_gis
npm install
```

Start PostgreSQL service and create the database:

```bash
sudo service postgresql start
psql -U postgres -c "CREATE DATABASE volcano_gis;"
psql -U postgres -d volcano_gis -c "CREATE EXTENSION postgis;"
```

## 📂 Prepare Data

Ensure the volcano data includes the following fields:

- 🌋 **Name:** Volcano name.
- 🗺️ **Location:** Latitude and longitude.
- 🔥 **Activity Status:** Active or inactive.
- 📏 **Radius:** Impact radius for buffer analysis.
- 📐 **Area:** Volcano's coverage area.

Import sample data with:

```bash
psql -U postgres -d volcano_gis -f ./data/volcano_data.sql
```

## 🌐 Application Features

1. **🗺️ Volcano Visualization:**
   - Interactive map displaying volcanoes across Europe.
   - Hovering over a volcano reveals details like name, location, activity status, radius, and area.

2. **🌋 Buffer Analysis:**
   - Calculates the affected area in case of an eruption based on the volcano's radius and area.
   - Displays the potential impact zone as a buffer around the volcano.

3. **⚠️ Risk Analysis:**
   - Determines if the user's location falls within the buffer zone.
   - If inside the buffer, calculates the risk percentage based on distance.
   - If outside, indicates the nearest volcano.

4. **🤖 Chatbot Integration:**
   - Answers general questions about volcanoes.
   - Predicts potential eruption times based on activity status and last eruption date.

## ▶️ Running the Application

1. **Start the server:**

```bash
node server.js
```

2. **Access the application via:**

```bash
http://localhost:3000
```

## 📓 Other Notes

- 🗺️ **Map Layer:** OpenLayers is used for volcano visualization.
- 🛠️ **Database Configuration:** Modify database connection settings in `server.js` if needed.
- ⏱️ **Performance:** Optimize queries for real-time responsiveness.

## ❗ Issues

Common issues and solutions:

- ⚠️ **Database Connection Failure:** Ensure PostgreSQL is running.
- ❌ **No Volcano Data Displayed:** Verify data import.
- 🤖 **Chatbot Not Responding:** Check API configuration.

## 🙏 Acknowledgements

Special thanks to TMMOB HKMO for support and guidance.

