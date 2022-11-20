// Copyright 2017 Silicon Laboratories, Inc.
//Sample Custom Payload Decoder Skeleton
//Users should use this as a framework to build their decoders. 

var addr = 0x00;

/*
This function will do the actual decoding of the payload. You should primarily 
use the functions as defined for the FieldContext object fc to do this. The functions
for event can be used to process information from earlier frames.
*/
function decode(event, fc) {
  fc.setAppendLengthsToFieldCodes(true);
  
  //PayloadDecoderLog.log("Decode.");

  addr = fc.decode("Addr", 1);
    
  if (addr == 0xFF)
  {
        fc.append("Cmd", 1);
		fc.append("Count", 4);
		fc.setSummary("Master packet");
  }
  else if ((addr <= 100) && (addr > 0))
  {
	    fc.append("Reserved", 1);
        fc.append("Count", 4);		
		fc.setSummary("Slave packet");
  }
  else
  {
		fc.setCorruption("Invalid Slave!");
		fc.setSummary("Invalid packet");
  }
}

//This function will determine if an event is relevant to this decoder.
//Note that this should be specific to the payload; if it is too generic, then
//event payloads belonging to other decoders may be mistakenly read and decoded by 
//this decoder.  
function accept(event, packet) {
  
  return true;
}

//This will let you set what the name of the frame should be when viewed in network
//analyzer. 
function name() {
	return "Slot frame decoder"
}

//This will determine what the decoder can be filtered by in network analyzer when
//viewing all packets. 
function filterName() {
	return "EFR32xG22"
}

//This is a function that is describing what the frame should have as its description 
//when you are viewing specific information in network analyzer
function frameDescription() {
	return "This is Slot frame decoder"
}


//This function will let you set whether the decode should be enabled by default when
//loading it for the first time, or when resetting the settings to default. 
function enabled() {
	return true;
}