def unicodetoascii(text):
  
    uni2ascii = {
        ord('\xe2\x80\x9c'.decode('utf-8')): ord('"'),
        ord('\xe2\x80\x99'.decode('utf-8')): ord("'"),
        ord('\xe2\x80\x93'.decode('utf-8')): ord("-"),
        ord('\xe9'.decode('latin1')): ord("e")

    }
    return text.decode('utf-8').translate(uni2ascii).encode('ascii')


