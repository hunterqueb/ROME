// ProportionalControl.pde
// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton, 
// depending on the value of the pot.
//
// Copyright (C) 2012 Mike McCauley
// $Id: ProportionalControl.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper(uint8_t interface = AccelStepper::FULL2WIRE, uint8_t pin1 = 0,uint8_t pin2 = 1, bool enamble = true); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

// This defines the analog input pin for reading the control voltage
// Tested with a 10k linear pot between 5v and GND

void setup()
{  
  stepper.setMaxSpeed(1000);
}

void loop()
{
  stepper.moveTo(1000);
  stepper.setSpeed(100);
  stepper.runSpeedToPosition();
}
