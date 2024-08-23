// Function to handle form submission
document.getElementById('login-form').addEventListener('submit', function(event) {
    event.preventDefault();
    
    // Trim and get entered IP address and port number
    const ipAddress = document.getElementById('ip-address').value.trim();
    const portNumber = document.getElementById('port-number').value.trim();

    // Check if entered IP address and port match the fixed values
    if (ipAddress === '192.168.7.5' && portNumber === '8080') {
        // Store the IP address and port number in session storage
        sessionStorage.setItem('ipAddress', ipAddress);
        sessionStorage.setItem('portNumber', portNumber);

        // Redirect to the dashboard page
        window.location.href = 'dashboard.html';
    } else {
        // Display error message for invalid IP address or port number
        document.getElementById('error-message').style.display = 'block';
    }
});

// Function to validate IP address format
function validateIPAddress(ipAddress) {
    const ipPattern = /^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$/;
    return ipPattern.test(ipAddress);
}

// Function to validate port number format
function validatePortNumber(portNumber) {
    const portNum = parseInt(portNumber, 10);
    return !isNaN(portNum) && portNum >= 1 && portNum <= 65535;
}

// Error handling for fetch operations
function handleFetchErrors(response) {
    if (!response.ok) {
        throw new Error('Network response was not ok');
    }
    return response.json();
}

// Function to fetch PLC status data
function fetchPLCStatus(ipAddress, portNumber) {
    fetch(`http://${ipAddress}:${portNumber}/status`)
        .then(handleFetchErrors)
        .then(data => {
            updateDashboard(data);
        })
        .catch(error => {
            console.error('Error fetching PLC status:', error);
            // Handle specific error case (e.g., display error message)
            // Example: document.getElementById('error-message').innerText = 'Failed to fetch PLC status';
        });
}

// Function to update dashboard based on received data
function updateDashboard(data) {
    // Update input status
    document.querySelector('input[name="x0"][value="' + (data.x0 ? 'on' : 'off') + '"]').checked = true;
    document.querySelector('input[name="x1"][value="' + (data.x1 ? 'on' : 'off') + '"]').checked = true;
    document.querySelector('input[name="x2"][value="' + (data.x2 ? 'on' : 'off') + '"]').checked = true;
    document.querySelector('input[name="x3"][value="' + (data.x3 ? 'on' : 'off') + '"]').checked = true;
    document.querySelector('input[name="x4"][value="' + (data.x4 ? 'on' : 'off') + '"]').checked = true;

    // Update output status
    document.querySelector('input[name="y0"][value="' + (data.y0 ? 'on' : 'off') + '"]').checked = true;
    document.querySelector('input[name="y1"][value="' + (data.y1 ? 'on' : 'off') + '"]').checked = true;
    document.querySelector('input[name="y2"][value="' + (data.y2 ? 'on' : 'off') + '"]').checked = true;
    document.querySelector('input[name="y3"][value="' + (data.y3 ? 'on' : 'off') + '"]').checked = true;
    document.querySelector('input[name="y4"][value="' + (data.y4 ? 'on' : 'off') + '"]').checked = true;

    // Update power indicator
    const powerIndicator = document.getElementById('power-on');
    powerIndicator.style.backgroundColor = data.power ? '#33cc33' : '#ccc';
}



function updateTime() {
    var date = new Date();
    var time = date.toLocaleTimeString();

    // Set the value of the textbox with id "current-time"
    document.getElementById('current-time').value = time;

    // Call requestAnimationFrame again to keep the time updated
    requestAnimationFrame(updateTime);
}

// Start the time updates
window.onload = function() {
    updateTime();
};


















































































































