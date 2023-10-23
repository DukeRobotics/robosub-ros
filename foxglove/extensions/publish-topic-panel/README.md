# Publish Topic Panel
This is an example panel to publish topics. Base a new extension off of this panel if it requires publishing topics.

## Usage
Enter the Topic Name, Schema Name, and Request in the appropriate input fields.

Ensure that the topic is prepended with a forwards slash.

Ensure that the request is formatted as a [JavaScript object](https://developer.mozilla.org/en-US/docs/Learn/JavaScript/Objects/JSON#json_structure) and conforms to the selected schema.

## Example
Topic Name: `/example_topic`

Schema Name: `std_msgs/String`

Request:
```js
{
  "data": "Example data"
}
```