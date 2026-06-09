	} else if (cmda == "SOP"){	//set flipper mirror (output path) to direct output (CCD)
		if (num==0){specrographError=ATSpectrographSetFlipperMirror(0,OUTPUT_FLIPPER,DIRECT);}
		if (num==1){specrographError=ATSpectrographSetFlipperMirror(0,OUTPUT_FLIPPER,SIDE);}
                if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetFlipperMirror(0,OUTPUT_FLIPPER,&fport);}
                val=std::to_string(fport);
	} else if (cmda == "GOP"){	//get flipper mirror output path
		specrographError=ATSpectrographGetFlipperMirror(0,OUTPUT_FLIPPER,&fport);
                val=std::to_string(fport);
	} else if (cmda == "SIP"){	//set flipper mirror (output path) to direct output (CCD)
		if (num==0){specrographError=ATSpectrographSetFlipperMirror(0,INPUT_FLIPPER,DIRECT);}
		if (num==1){specrographError=ATSpectrographSetFlipperMirror(0,INPUT_FLIPPER,SIDE);}
                if(specrographError==ATSPECTROGRAPH_SUCCESS){specrographError=ATSpectrographGetFlipperMirror(0,INPUT_FLIPPER,&ifport);}
                val=std::to_string(ifport);
	} else if (cmda == "GIP"){	//get flipper mirror output path
		specrographError=ATSpectrographGetFlipperMirror(0,INPUT_FLIPPER,&ifport);
                val=std::to_string(ifport);
	} else if (cmda == "SGR"){	//set grating
