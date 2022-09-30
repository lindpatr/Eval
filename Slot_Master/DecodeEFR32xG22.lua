local decoder = {}
 
decoder.name = "EFR32xG22 Test Decoderx	"
decoder.filterName = "luaEFR32xG33"
decoder.description = "Lua decoder for Belenos EFR32xG22 EV card"
 
-- Init function is called whenever the decoder is created the first time.
-- It is NOT called for every packet. So only do global initialization here.
function decoder.init()
FRAME_CUSTOMPAYLOAD = 6
end
 
-- Accept is called for EVERY packet. if Accept returns true, then decode is called.
function decoder.accept()
  log("Accept.")
  if payloadByte(0) == 0x00 then
    return false
  else
    return true
  end
end
 
-- Decode function is called for EVERY packet that is accepted.
function decoder.decode()
  log("Decode.")
  local byte0 = decode("Addr", 1)
  
  if byte0 == 0xFF then
      append("Cmd", 1)
  else
       append("Reserved", 1) 
  end   
  
  appendBytes("Count", 4)

  setSummary("Decoded packet")
end
 
return decoder