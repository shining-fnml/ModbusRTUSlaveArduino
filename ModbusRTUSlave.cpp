#include "ModbusRTUSlave.h"
#include "utility/LinkedList.h"


ModbusRTUSlave::ModbusRTUSlave(byte const slaveAddress, HardwareSerial* serialport, u8 const controlPinArg) :
  slave(slaveAddress),
  ser(serialport),
  controlPin(controlPinArg),
  isReading(true),
  words(new LinkedList<ModbusRTUSlaveAddress*>()),
  inputs(new LinkedList<ModbusRTUSlaveAddress*>()),
  bits(new LinkedList<ModbusRTUSlaveAddress*>()),
  discretes(new LinkedList<ModbusRTUSlaveAddress*>())
{
  pinMode(this->controlPin, OUTPUT);
  digitalWrite(this->controlPin, LOW);
}

void ModbusRTUSlave::begin(int baudrate, word mode) 
{
	ser->begin(baudrate, mode);
	ResCnt=0;
}

void ModbusRTUSlave::setSerial(int baudrate, word mode)
{
	ser->flush();
	ser->begin(baudrate, mode);
	ResCnt=0;
}

boolean ModbusRTUSlave::addHoldingArea(uint16_t Address, uint16_t* values, size_t cnt)
{
	return addWordArea(Address, values, cnt, ADDRESS_TYPE_HOLDING, words);
}

boolean ModbusRTUSlave::addInputArea(uint16_t Address, uint16_t* values, size_t cnt)
{
	return addWordArea(Address, values, cnt, ADDRESS_TYPE_INPUT, inputs);
}

boolean ModbusRTUSlave::addWordArea(uint16_t Address, uint16_t *values, size_t cnt, char areatype, LinkedList<ModbusRTUSlaveAddress *> *area)
{
	for(int i = 0; i < area->size(); i++) {
		ModbusRTUSlaveAddress *a = area->get(i);

		if(a!=NULL && Address >= a->addr && Address+cnt <= a->addr+a->len)
			return false;
	}
	area->add(new ModbusRTUSlaveWordAddress(Address, values, cnt, areatype));
	return true;
}

boolean ModbusRTUSlave::addCoilArea(uint16_t Address, uint8_t* values, size_t cnt)
{
	return addBitArea(Address, values, cnt, ADDRESS_TYPE_COIL, bits);
}

boolean ModbusRTUSlave::addDiscreteArea(uint16_t Address, uint8_t* values, size_t cnt)
{
	return addBitArea(Address, values, cnt, ADDRESS_TYPE_DISCRETE, discretes);
}

boolean ModbusRTUSlave::addBitArea(uint16_t Address, uint8_t* values, size_t cnt, char areatype, LinkedList<ModbusRTUSlaveAddress *> *area)
{
	if(getAddress<ModbusRTUSlaveBitAddress *>(area, Address, cnt) == NULL) {
		area->add(new ModbusRTUSlaveBitAddress(Address, values, cnt, areatype));
		return true;
	}
	return false;
}

template <typename T>
T ModbusRTUSlave::getAddress(LinkedList<ModbusRTUSlaveAddress*> *area, uint16_t Addr, uint16_t Len)
{
	for(int i = 0; i < area->size(); i++) {
		T a = area->get(i);
		if(a!=NULL && Addr >= a->addr && Addr+Len <= a->addr + a->len)
			return a;
	}
	return NULL;
}

void ModbusRTUSlave::readBits(LinkedList<ModbusRTUSlaveAddress *> *area, uint16_t Address, byte Slave, byte Function, bool *bvalid)
{
	if(ResCnt < 8)
		return;
	u16 Length = (lstResponse[4] << 8) | lstResponse[5];
	byte hi = 0xFF, lo = 0xFF;
	getCRC(lstResponse,300, 0, 6, &hi, &lo);
	ModbusRTUSlaveBitAddress *a = getBitArea(area, Address, Length);
	if (!(Length > 0 && a != NULL && lstResponse[6] == hi && lstResponse[7] == lo)) {
		*bvalid = false;
		return;
	}
	u16 stidx = (Address - a->addr) / 8;
	u16 nlen = ((Length-1) / 8)+1;

	byte dat[nlen];
	memset(dat,0,nlen);

	int ng=(Address - a->addr) % 8;
	int ns=stidx;
	for(int i=0;i<nlen;i++) {
		byte val=0;
		for(int j=0;j<8;j++) {
			if(bitRead(a->values[ns], ng++))
				bitSet(val,j);
			if(ng==8) {
				ns++;
				ng=0;
			}
		}
		dat[i]=val;
	}

	byte ret[3+nlen+2];
	ret[0]=Slave;	ret[1]=Function;	ret[2]=nlen;
	for(int i=0;i<nlen;i++)
		ret[3+i]=dat[i];
	hi = 0xFF;
	lo = 0xFF;
	getCRC(ret, 3+nlen+2, 0, 3+nlen, &hi, &lo);
	ret[3+nlen]=hi;
	ret[3+nlen+1]=lo;
	doWrite(ret, 3+nlen+2);
	ResCnt=0;
}

void ModbusRTUSlave::readWords(LinkedList<ModbusRTUSlaveAddress *> *area, uint16_t Address, byte Slave, byte Function, bool *bvalid)
{
	if(ResCnt < 8)
		return;
	u16 Length = (lstResponse[4] << 8) | lstResponse[5];
	byte hi = 0xFF, lo = 0xFF;
	getCRC(lstResponse,300, 0, 6, &hi, &lo);
	ModbusRTUSlaveWordAddress *a = getAddress<ModbusRTUSlaveWordAddress *>(area, Address, Length);
	if (!(Length > 0 && a != NULL && lstResponse[6] == hi && lstResponse[7] == lo)) {
		*bvalid = false;
		return;
	}
	u16 stidx = Address - a->addr;
	u16 nlen = Length * 2;

	byte ret[3+nlen+2];
	ret[0]=Slave;	ret[1]=Function;	ret[2]=nlen;
	for(int i=stidx;i<stidx+Length;i++) {
		ret[3+((i-stidx)*2)+0]=((a->values[i] & 0xFF00) >> 8);
		ret[3+((i-stidx)*2)+1]=((a->values[i] & 0xFF));
	}
	hi = 0xFF;
	lo = 0xFF;
	getCRC(ret, 3+nlen+2, 0, 3+nlen, &hi, &lo);
	ret[3+nlen]=hi;
	ret[3+nlen+1]=lo;
	doWrite(ret, 3+nlen+2);
	ResCnt=0;
}

ModbusRTUSlaveAddress *ModbusRTUSlave::process()
{
	bool bvalid = true;
	ModbusRTUSlaveAddress *modifiedArea = NULL;

	while(this->isDataAvail()) 
	{
		byte d = this->doRead();

		lstResponse[ResCnt++]=d;
		if(ResCnt>=4)
		{
			byte Slave = lstResponse[0];
			if(Slave == slave)
			{
				byte Function = lstResponse[1];
				u16 Address = (lstResponse[2] << 8) | lstResponse[3];
				switch(Function)
				{
					case 1:		//BitRead
						readBits(bits, Address, Slave, Function, &bvalid);
					case 2:
						readBits(discretes, Address, Slave, Function, &bvalid);
						break;
					case 3:		//WordRead	
						readWords(words, Address, Slave, Function, &bvalid);
						break;
					case 4:
						readWords(inputs, Address, Slave, Function, &bvalid);
						break;
					case 5:		//BitWrite
						if(ResCnt >= 8)
						{
							u16 Data = (lstResponse[4] << 8) | lstResponse[5];
							byte hi = 0xFF, lo = 0xFF;
							getCRC(lstResponse,300, 0, 6, &hi, &lo);
							ModbusRTUSlaveBitAddress *a = getCoilArea(Address);
							if (a != NULL && lstResponse[6] == hi && lstResponse[7] == lo)
							{
								u16 stidx = (Address - a->addr) / 8;

								bitWrite(a->values[stidx], (Address - a->addr)%8, Data==0xFF00);

								byte ret[8];
								ret[0]=Slave;	
								ret[1]=Function;	
								ret[2]=((Address&0xFF00)>>8);
								ret[3]=((Address&0x00FF));
								ret[4]=((Data&0xFF00)>>8);
								ret[5]=((Data&0x00FF));
								byte hi = 0xFF, lo = 0xFF;
								getCRC(ret, 8, 0, 6, &hi, &lo);
								ret[6]=hi;
								ret[7]=lo;
								doWrite(ret, 8);
								ResCnt=0;
								modifiedArea = a;
							}
							else bvalid = false;
						}
						break;
					case 6:		//WordWrite
						if(ResCnt >= 8)
						{
							u16 Data = (lstResponse[4] << 8) | lstResponse[5];
							byte hi = 0xFF, lo = 0xFF;
							getCRC(lstResponse,300, 0, 6, &hi, &lo);
							ModbusRTUSlaveWordAddress *a = getWordArea(Address);
							if (a != NULL && lstResponse[6] == hi && lstResponse[7] == lo)
							{
								u16 stidx = Address - a->addr;

								a->values[stidx] = Data;

								byte ret[8];
								ret[0]=Slave;	
								ret[1]=Function;	
								ret[2]=((Address&0xFF00)>>8);
								ret[3]=((Address&0x00FF));
								ret[4]=((Data&0xFF00)>>8);
								ret[5]=((Data&0x00FF));
								byte hi = 0xFF, lo = 0xFF;
								getCRC(ret, 8, 0, 6, &hi, &lo);
								ret[6]=hi;
								ret[7]=lo;
								doWrite(ret, 8);
								modifiedArea = a;
								ResCnt=0;
							}
							else bvalid = false;
						}
						break;
					case 15:	//MultiBitWrite
						if(ResCnt >= 7)
						{
							u16 Length = (lstResponse[4] << 8) | lstResponse[5];
							u8 ByteCount = lstResponse[6];
							if(ResCnt >= 9+ByteCount)
							{
								byte hi = 0xFF, lo = 0xFF;
								getCRC(lstResponse,300, 0, 7 + ByteCount, &hi, &lo);
								if(lstResponse[(9 + ByteCount - 2)] == hi && lstResponse[(9 + ByteCount - 1)] == lo)
								{
									ModbusRTUSlaveBitAddress *a = getCoilArea(Address, Length);
									if (a != NULL) 
									{
										u16 stidx = (Address - a->addr) / 8;
										int ng=(Address - a->addr) % 8;
										int ns=stidx;

										for(int i=7; i<7+ByteCount;i++)
										{
											byte val = lstResponse[i];
											for(int j=0;j<8;j++)
											{
												bitWrite(a->values[ns], ng++, bitRead(val,j));
												if(ng==8){ns++;ng=0;}
											}
										}

										if(bvalid)
										{
											byte ret[8];
											ret[0]=Slave;	
											ret[1]=Function;	
											ret[2]=((Address&0xFF00)>>8);
											ret[3]=((Address&0x00FF));
											ret[4]=((Length&0xFF00)>>8);
											ret[5]=((Length&0x00FF));
											byte hi = 0xFF, lo = 0xFF;
											getCRC(ret, 8, 0, 6, &hi, &lo);
											ret[6]=hi;
											ret[7]=lo;
											this->doWrite(ret, 8);
											modifiedArea = a;
											ResCnt=0;
										}
									}
								}
								else bvalid=false;
							}
						}
						break; 
					case 16:	//MultiWordWrite
						if(ResCnt >= 7)
						{
							u16 Length = (lstResponse[4] << 8) | lstResponse[5];
							u8 ByteCount = lstResponse[6];
							if(ResCnt >= 9+ByteCount)
							{
								byte hi = 0xFF, lo = 0xFF;
								getCRC(lstResponse,300, 0, 7 + ByteCount, &hi, &lo);
								if(lstResponse[(9 + ByteCount - 2)] == hi && lstResponse[(9 + ByteCount - 1)] == lo)
								{
									modifiedArea = getWordArea(Address);
									for(int i=7; i<7+ByteCount;i+=2)
									{
										u16 data = lstResponse[i] << 8 | lstResponse[i+1];
										ModbusRTUSlaveWordAddress *a = getWordArea(Address + ((i-7)/2));
										if (a != NULL) { a->values[(Address + ((i-7)/2)) - a->addr] = data;	}
										else { bvalid=false; break; }
									}
									if(bvalid)
									{
										byte ret[8];
										ret[0]=Slave;	
										ret[1]=Function;	
										ret[2]=((Address&0xFF00)>>8);
										ret[3]=((Address&0x00FF));
										ret[4]=((Length&0xFF00)>>8);
										ret[5]=((Length&0x00FF));
										byte hi = 0xFF, lo = 0xFF;
										getCRC(ret, 8, 0, 6, &hi, &lo);
										ret[6]=hi;
										ret[7]=lo;
										doWrite(ret, 8);
										ResCnt=0;
									}
								}
								else bvalid=false;
							}
						}
						break;
				}
			}
			else bvalid = false;
		}
		lastrecv = millis();
	}
	if(!bvalid && ResCnt>0) ResCnt=0;
	if(ResCnt>0 && (millis()-lastrecv > 200 || millis() < lastrecv)) ResCnt=0;
	return modifiedArea;
}

static void ModbusRTUSlave::getCRC(byte* pby, int arsize, int startindex, int nSize, byte* byFirstReturn, byte* bySecondReturn)
{
	int uIndex;
	byte uchCRCHi = 0xff;
	byte uchCRCLo = 0xff;
	for (int i = startindex; i < startindex + nSize && i<arsize; i++)
	{
		uIndex = uchCRCHi ^ pby[i];
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	(*byFirstReturn) = uchCRCHi;
	(*bySecondReturn) = uchCRCLo;
}

void ModbusRTUSlave::switchToReadingIfNotReadingNow()
{
  if (not this->isReading)
	{
	  this->ser->flush();
	  digitalWrite(this->controlPin, LOW);
	  this->isReading = true;
	}
}

bool ModbusRTUSlave::isDataAvail()
{
  this->switchToReadingIfNotReadingNow();
  return this->ser->available();
}

int ModbusRTUSlave::doRead()
{
  this->switchToReadingIfNotReadingNow();
  return ser->read();
}

void ModbusRTUSlave::doWrite(byte* buffer, int const length)
{
  if (this->isReading)
	{
	  digitalWrite(this->controlPin, HIGH);
	  this->isReading = false;
	}
  this->ser->write(buffer, length);
}

void ModbusRTUSlave::setSlave(byte slaveId)
{
	slave = slaveId;
}

boolean getBit(u8* area, int index)
{
	u16 stidx = index / 8;
	return bitRead(area[stidx], index%8);
}

void setBit(u8* area, int index, bool value)
{
	u16 stidx = index / 8;
	bitWrite(area[stidx], index%8, value);
}

ModbusRTUSlaveBitAddress::ModbusRTUSlaveBitAddress(u16 Address, u8* value, size_t cnt, char adrtype)
{
	addr = Address;
	values = value;
	len = cnt;
	type = adrtype;
}

ModbusRTUSlaveWordAddress::ModbusRTUSlaveWordAddress(uint16_t Address, uint16_t* value, size_t cnt, char adrtype)
{
	addr = Address;
	values = value;
	len = cnt;
	type = adrtype;
}
