import cv2
from pyzbar import pyzbar


def read_QR():
    cp = cv2.VideoCapture(0)
    while True:
        _, frame = cp.read()
        decoded_QR = pyzbar.decode(frame)
        for qr in decoded_QR:
            try:
                name = qr.data.decode("utf-8")
                if name == 'James':
                    return True, "James"
                elif name == 'Gorkem':
                    return True, "Gorkem"
                else:
                    print(f"unrecognised QR code {name}")
                    return False, "unknown"

            except Exception as e:
                print(e, qr.data)
                return False, "unknown"

if __name__ == '__main__':
    print(read_QR())