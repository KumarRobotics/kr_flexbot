import hmac, hashlib, socket, time
SECRET=b"CHANGE_ME_TO_RANDOM_LONG_SECRET"
cmd="ARM"
ts=str(int(time.time()))
base=f"{cmd}|{ts}".encode()
sig=hmac.new(SECRET, base, hashlib.sha256).hexdigest()
msg=f"{cmd}|{ts}|{sig}".encode()
sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(msg, ("192.168.0.2", 5000))
print(msg.decode())