## to run the code: 

ssh into sawyer (alice)
`cd ~/ros_workspaces/106a-final`
`rosrun code robot_code.py`

## to run the server: (only ria's computer has access rn)
```
ssh honeydew
cd 106a-final/vision
python server.py
```

(in a separate window)
```
ngrok http 8000
```

then replace the URL in client.py with the one displayed by ngrok