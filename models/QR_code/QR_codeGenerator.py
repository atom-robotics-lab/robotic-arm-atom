# Import QRCode from pyqrcode
import pyqrcode
import png
from pyqrcode import QRCode
  
  
# String which represents the QR code
baseString = "Customer - "
  
for i in range(1,31) :
    stringPrint = baseString + str(i)
    url = pyqrcode.create(stringPrint)
    url.png( baseString+ str(i) + '.png', scale = 8)

