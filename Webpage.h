#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <MultiStepper.h>

#include <Arduino.h>

const char PAGE_MAIN[] PROGMEM = R"=====(


<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Robot Controller</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: #f4f7f6;
            color: #333;
            margin: 0;
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        .container {
            background-color: #fff;
            border-radius: 12px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
            padding: 25px;
            margin-bottom: 25px;
            width: 100%;
            max-width: 800px;
            box-sizing: border-box;
        }
        h1, h2 {
            color: #2c3e50;
            text-align: center;
            margin-top: 10px;
            margin-bottom: 20px;
        }
        .section-title {
            border-bottom: 2px solid #3498db;
            padding-bottom: 5px;
            margin-bottom: 15px;
            font-size: 1.3em;
            color: #34495e;
        }
        .control-group {
            margin-bottom: 20px;
            padding: 15px;
            border: 1px solid #e0e0e0;
            border-radius: 8px;
            background-color: #fafafa;
        }
        .control-group p {
            margin-bottom: 10px;
            font-size: 0.95em;
        }
        .btn-group {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            justify-content: center;
            margin-top: 10px;
        }
        button {
            background-color: #3498db;
            color: white;
            border: none;
            border-radius: 6px;
            padding: 12px 20px;
            font-size: 1.0em;
            cursor: pointer;
            transition: background-color 0.3s ease, transform 0.2s ease;
            flex-grow: 1;
            max-width: 200px;
        }
        button:hover {
            background-color: #2980b9;
            transform: translateY(-2px);
        }
        button:active {
            transform: translateY(0);
        }
        .btn-danger {
            background-color: #e74c3c;
        }
        .btn-danger:hover {
            background-color: #c0392b;
        }
        .status-display {
            background-color: #ecf0f1;
            border: 1px solid #dcdcdc;
            border-radius: 8px;
            padding: 15px;
            font-family: 'Consolas', 'Monaco', monospace;
            white-space: pre-wrap;
            word-break: break-all;
            max-height: 250px;
            overflow-y: auto;
            text-align: left;
            font-size: 0.9em;
            color: #555;
        }
        .highlight {
            font-weight: bold;
            color: #27ae60;
        }
        .motor-pos {
            font-weight: bold;
            color: #e67e22;
        }
        @media (max-width: 600px) {
            body {
                padding: 10px;
            }
            .container {
                padding: 15px;
            }
            button {
                width: 100%;
                max-width: none;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Robot Arm Controller</h1>
        <p style="text-align: center; font-size: 1.1em; color: #555;">Control your robotic arm via Wi-Fi!</p>
    </div>

    <div class="container">
        <h2 class="section-title">Control Mode Selection</h2>
        <div class="control-group">
            <p>Current Control Mode: <span id="current_mode" class="highlight">Fetching...</span></p>
            <div class="btn-group">
                <button onclick="sendCommand('/MODE?value=1')">Set Mode 1 (Web/Serial)</button>
                <button onclick="sendCommand('/MODE?value=2')">Set Mode 2 (PS4)</button>
            </div>
            <p style="font-size:0.85em; margin-top: 15px; color:#777;"><em>Mode 1: Commands from web page or serial input. Mode 2: Commands from PS4 controller.</em></p>
        </div>
    </div>

    <div class="container">
        <h2 class="section-title">Motor Position Control (Mode 1 Only)</h2>
        <p style="text-align: center; color: #e74c3c; font-weight: bold;">Manual control only works in Mode 1.</p>
        
        <div class="control-group">
            <p><strong>Motor 1 (X-Axis):</strong> Position: <span id="pos1_output" class="motor-pos">0</span> steps</p>
            <div class="btn-group">
                <button onclick="sendCommand('/MOVE?m=1&s=1000')">M1 +1000</button>
                <button onclick="sendCommand('/MOVE?m=1&s=-1000')">M1 -1000</button>
                <button onclick="sendCommand('/MOVE?m=1&s=5000')">M1 +5000</button>
                <button onclick="sendCommand('/MOVE?m=1&s=-5000')">M1 -5000</button>
            </div>
        </div>

        <div class="control-group">
            <p><strong>Motor 2 (Y-Axis):</strong> Position: <span id="pos2_output" class="motor-pos">0</span> steps</p>
            <div class="btn-group">
                <button onclick="sendCommand('/MOVE?m=2&s=1000')">M2 +1000</button>
                <button onclick="sendCommand('/MOVE?m=2&s=-1000')">M2 -1000</button>
                <button onclick="sendCommand('/MOVE?m=2&s=5000')">M2 +5000</button>
                <button onclick="sendCommand('/MOVE?m=2&s=-5000')">M2 -5000</button>
            </div>
        </div>

        <div class="control-group">
            <p><strong>Motor 3 (Z-Axis):</strong> Position: <span id="pos3_output" class="motor-pos">0</span> steps</p>
            <div class="btn-group">
                <button onclick="sendCommand('/MOVE?m=3&s=1000')">M3 +1000</button>
                <button onclick="sendCommand('/MOVE?m=3&s=-1000')">M3 -1000</button>
                <button onclick="sendCommand('/MOVE?m=3&s=5000')">M3 +5000</button>
                <button onclick="sendCommand('/MOVE?m=3&s=-5000')">M3 -5000</button>
            </div>
        </div>

        <div class="control-group">
            <p><strong>Motor 4 (Rotation):</strong> Position: <span id="pos4_output" class="motor-pos">0</span> steps</p>
            <div class="btn-group">
                <button onclick="sendCommand('/MOVE?m=4&s=1000')">M4 +1000</button>
                <button onclick="sendCommand('/MOVE?m=4&s=-1000')">M4 -1000</button>
                <button onclick="sendCommand('/MOVE?m=4&s=5000')">M4 +5000</button>
                <button onclick="sendCommand('/MOVE?m=4&s=-5000')">M4 -5000</button>
            </div>
        </div>
    </div>

    <div class="container">
        <h2 class="section-title">Real-time Status</h2>
        <div class="status-display" id="status_output">
            Fetching status...
        </div>
    </div>

    <script>
        const updateInterval = 1000; // Update every 1 second
        const motorSteps = 1000; // Default steps for manual move buttons

        // Function to send a command to the ESP32
        function sendCommand(command) {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    console.log("Command response:", this.responseText);
                    // No direct action, as getStatus will update everything
                } else if (this.readyState == 4 && this.status != 200) {
                    console.error("Command failed: ", this.status, this.responseText);
                    alert("Command failed: " + this.responseText);
                }
            };
            xhttp.open("GET", command, true);
            xhttp.send();
            setTimeout(getStatus, 100); // Request status update shortly after sending command
        }

        // Function to fetch and update status from the ESP32
        function getStatus() {
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
                if (this.readyState == 4 && this.status == 200) {
                    try {
                        var parser = new DOMParser();
                        var xmlDoc = parser.parseFromString(this.responseText, "text/xml");

                        // Update Control Mode
                        var modeElement = xmlDoc.getElementsByTagName("MODE")[0];
                        if (modeElement) {
                            let mode = modeElement.childNodes[0].nodeValue;
                            document.getElementById("current_mode").innerHTML = mode + (mode == "1" ? " (Web/Serial)" : " (PS4)");
                        }

                        // Update Motor Positions
                        var pos1Element = xmlDoc.getElementsByTagName("POS1")[0];
                        if (pos1Element) document.getElementById("pos1_output").innerHTML = pos1Element.childNodes[0].nodeValue;
                        var pos2Element = xmlDoc.getElementsByTagName("POS2")[0];
                        if (pos2Element) document.getElementById("pos2_output").innerHTML = pos2Element.childNodes[0].nodeValue;
                        var pos3Element = xmlDoc.getElementsByTagName("POS3")[0];
                        if (pos3Element) document.getElementById("pos3_output").innerHTML = pos3Element.childNodes[0].nodeValue;
                        var pos4Element = xmlDoc.getElementsByTagName("POS4")[0];
                        if (pos4Element) document.getElementById("pos4_output").innerHTML = pos4Element.childNodes[0].nodeValue;

                        // Display raw XML for debugging/general status
                        document.getElementById("status_output").innerText = this.responseText;

                    } catch (e) {
                        console.error("Error parsing XML:", e);
                        document.getElementById("status_output").innerText = "Error parsing XML: " + e.message + "\n" + this.responseText;
                    }
                } else if (this.readyState == 4 && this.status != 200) {
                    console.error("Failed to fetch status: ", this.status);
                    document.getElementById("status_output").innerText = "Failed to fetch status. HTTP Status: " + this.status;
                }
            };
            xhttp.open("GET", "/xml", true); // Request XML data from /xml endpoint
            xhttp.send();
        }

        // Start polling for status updates
        setInterval(getStatus, updateInterval);
        // Initial status fetch on page load
        getStatus();

    </script>
</body>
</html>
)=====";

#endif // WEBPAGE_H