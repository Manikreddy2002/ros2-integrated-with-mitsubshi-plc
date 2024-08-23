
// Import required modules
const opcua = require("node-opcua");

// Define OPC UA server endpoint URL
const endpointUrl = 'opc.tcp://localhost:4840'; // Example OPC UA server endpoint URL

// Function to fetch PLC status from OPC UA
function fetchPLCStatusFromOPCUA() {
    // Create OPC UA client and connect
    const client = new opcua.OPCUAClient({});
    
    client.connect(endpointUrl, function (err) {
        if (err) {
            console.error("Error connecting to OPC UA server:", err.message);
            return;
        }

        client.createSession(function (err, session) {
            if (err) {
                console.error("Error creating session with OPC UA server:", err.message);
                client.disconnect();
                return;
            }

            const nodesToRead = [
                { nodeId: 'ns=1;s=x0', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=x1', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=x2', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=x3', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=x4', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=y0', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=y1', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=y2', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=y3', attributeId: opcua.AttributeIds.Value },
                { nodeId: 'ns=1;s=y4', attributeId: opcua.AttributeIds.Value },
            ];

            session.read(nodesToRead, function (err, dataValues) {
                if (err) {
                    console.error("Error reading OPC UA data:", err.message);
                } else {
                    const data = {
                        x0: dataValues[0].value.value,
                        x1: dataValues[1].value.value,
                        x2: dataValues[2].value.value,
                        x3: dataValues[3].value.value,
                        x4: dataValues[4].value.value,
                        y0: dataValues[5].value.value,
                        y1: dataValues[6].value.value,
                        y2: dataValues[7].value.value,
                        y3: dataValues[8].value.value,
                        y4: dataValues[9].value.value,
                    };

                    updateDashboard(data); // Update dashboard with fetched data
                }

                session.close(function (err) {
                    if (err) {
                        console.error("Error closing OPC UA session:", err.message);
                    }
                    client.disconnect(function () {
                        console.log("Disconnected from OPC UA server");
                    });
                });
            });
        });
    });
}

// Function to handle OPC UA client events for writing and reading
async function performOPCUAOperations() {
    try {
        // Connect to OPC UA server
        const client = opcua.OPCUAClient.create({
            endpoint_must_exist: false // Allow connection even if endpoint doesn't exist
        });
        await client.connect(endpointUrl);
        console.log("Connected to OPC UA server");

        // Create session
        const session = await client.createSession();
        console.log("Session created");

        // Write example data to a node
        const nodeIdToWrite = 'ns=1;s=MyVariableToWrite'; // Example node ID for writing
        const newValue = { dataType: opcua.DataType.Int32, value: 456 }; // Example value to write
        await session.writeSingleNode(nodeIdToWrite, newValue);
        console.log(`Successfully wrote value ${newValue.value} to node ${nodeIdToWrite}`);

        // Read example data from another node
        const nodeIdToRead = 'ns=1;s=MyVariableToRead'; // Example node ID for reading
        const dataValue = await session.readVariableValue(nodeIdToRead);
        console.log(`Read value ${dataValue.value.value} from node ${nodeIdToRead}`);

        // Close session
        await session.close();
        console.log("Session closed");

        // Disconnect from OPC UA server
        await client.disconnect();
        console.log("Disconnected from OPC UA server");

    } catch (err) {
        console.error("Error:", err.message);
    } finally {
        // Ensure client disconnects on error or success
        await client.disconnect();
    }
}

// Example function to update dashboard with fetched data
function updateDashboard(data) {
    // Implement your dashboard update logic here
    console.log("Updating dashboard with data:", data);
}
