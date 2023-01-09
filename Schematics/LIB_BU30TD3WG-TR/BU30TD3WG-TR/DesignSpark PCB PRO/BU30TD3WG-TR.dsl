SamacSys ECAD Model
910776/1049003/2.49/5/3/Power Supply

DESIGNSPARK_INTERMEDIATE_ASCII

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "r90_60"
		(holeDiam 0)
		(padShape (layerNumRef 1) (padShapeType Rect)  (shapeWidth 0.6) (shapeHeight 0.9))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 0) (shapeHeight 0))
	)
	(textStyleDef "Default"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 50 mils)
			(strokeWidth 5 mils)
		)
	)
	(patternDef "SOT95P280X125-5N" (originalName "SOT95P280X125-5N")
		(multiLayer
			(pad (padNum 1) (padStyleRef r90_60) (pt -1.4, 0.95) (rotation 90))
			(pad (padNum 2) (padStyleRef r90_60) (pt -1.4, 0) (rotation 90))
			(pad (padNum 3) (padStyleRef r90_60) (pt -1.4, -0.95) (rotation 90))
			(pad (padNum 4) (padStyleRef r90_60) (pt 1.4, -0.95) (rotation 90))
			(pad (padNum 5) (padStyleRef r90_60) (pt 1.4, 0.95) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0, 0) (textStyleRef "Default") (isVisible True))
		)
		(layerContents (layerNumRef 30)
			(line (pt -2.1 1.8) (pt 2.1 1.8) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 2.1 1.8) (pt 2.1 -1.8) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt 2.1 -1.8) (pt -2.1 -1.8) (width 0.05))
		)
		(layerContents (layerNumRef 30)
			(line (pt -2.1 -1.8) (pt -2.1 1.8) (width 0.05))
		)
		(layerContents (layerNumRef 28)
			(line (pt -0.825 1.45) (pt 0.825 1.45) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 0.825 1.45) (pt 0.825 -1.45) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt 0.825 -1.45) (pt -0.825 -1.45) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -0.825 -1.45) (pt -0.825 1.45) (width 0.1))
		)
		(layerContents (layerNumRef 28)
			(line (pt -0.825 0.5) (pt 0.125 1.45) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.6 1.45) (pt 0.6 1.45) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0.6 1.45) (pt 0.6 -1.45) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt 0.6 -1.45) (pt -0.6 -1.45) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -0.6 -1.45) (pt -0.6 1.45) (width 0.2))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1.85 1.5) (pt -0.95 1.5) (width 0.2))
		)
	)
	(symbolDef "BU30TD3WG-TR" (originalName "BU30TD3WG-TR")

		(pin (pinNum 1) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 2) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 3) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Default"))
		))
		(pin (pinNum 4) (pt 1100 mils 0 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -25 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(pin (pinNum 5) (pt 1100 mils -100 mils) (rotation 180) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 870 mils -125 mils) (rotation 0]) (justify "Right") (textStyleRef "Default"))
		))
		(line (pt 200 mils 100 mils) (pt 900 mils 100 mils) (width 6 mils))
		(line (pt 900 mils 100 mils) (pt 900 mils -300 mils) (width 6 mils))
		(line (pt 900 mils -300 mils) (pt 200 mils -300 mils) (width 6 mils))
		(line (pt 200 mils -300 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 950 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Default"))

	)
	(compDef "BU30TD3WG-TR" (originalName "BU30TD3WG-TR") (compHeader (numPins 5) (numParts 1) (refDesPrefix PS)
		)
		(compPin "1" (pinName "VIN") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "2" (pinName "GND") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "3" (pinName "STBY") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "4" (pinName "N.C.") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(compPin "5" (pinName "VOUT") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Bidirectional))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "BU30TD3WG-TR"))
		(attachedPattern (patternNum 1) (patternName "SOT95P280X125-5N")
			(numPads 5)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "4")
				(padNum 5) (compPinRef "5")
			)
		)
		(attr "Manufacturer_Name" "ROHM Semiconductor")
		(attr "Manufacturer_Part_Number" "BU30TD3WG-TR")
		(attr "Mouser Part Number" "755-BU30TD3WG-TR")
		(attr "Mouser Price/Stock" "https://www.mouser.com/Search/Refine.aspx?Keyword=755-BU30TD3WG-TR")
		(attr "Arrow Part Number" "")
		(attr "Arrow Price/Stock" "")
		(attr "Mouser Testing Part Number" "")
		(attr "Mouser Testing Price/Stock" "")
		(attr "Description" "ROHM - BU30TD3WG-TR - LDO, FIXED, 3V, 0.2A, SSOP-5")
		(attr "Datasheet Link" "https://componentsearchengine.com/Datasheets/2/BU30TD3WG-TR.pdf")
		(attr "Height" "1.25 mm")
	)

)
