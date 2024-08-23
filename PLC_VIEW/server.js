const express = require('express');
const rclnodejs = require('rclnodejs');

const app = express();
const port = 3000;

app.use(express.static('public'));

app.get('/status', (req, res) => {
  // Replace with your ROS2 logic
  rclnodejs.init().then(() => {
    const node = new rclnodejs.Node('status_node');
    const client = node.createClient('your_service_type', 'your_service_name');

    client.sendRequest({}).then((response) => {
      res.json(response);
    }).catch((error) => {
      res.status(500).send(error.message);
    });
  });
});

app.listen(port, () => {
  console.log(`Server running at http://localhost:${port}`);
});

