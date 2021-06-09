module.exports = {
  credentialSecret: 'lpwan-workshop',
  adminAuth: {
    type: "credentials",
    prompts: [
      {
        "id": "username",
        "type": "text",
        "label": "Username"
      },
      {
        "id": "password",
        "type": "password",
        "label": "Password"
      }
    ],
    users: [
      {
        username: "admin",
        password: "$2a$08$5NwW50Pa2TwtYFsmIGbFyO85p1nS10M738kIdP7KfFLUxQqRCysIa",
        permissions: "*"
      }
    ]
  }
}