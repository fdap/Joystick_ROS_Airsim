import inputs

while True:
  events = inputs.get_gamepad()
  for event in events:
    print(event.code, event.state)