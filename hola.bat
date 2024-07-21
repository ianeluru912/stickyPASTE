(echo YO=%1
echo/
echo OTRO=%2) > "src\persona.py"
stickytape src/main.py --add-python-path src --output-file c%1.py