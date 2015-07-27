#ifndef CANPLUS_HPP
#define CANPLUS_HPP

/*******************************************
 * canplus ver1.0 2015/3/3
 * Classes of Controller Area Network communication for stm32f4 discovery 2-layer board.
 * CanInterface declaration.
 *
 * [Dependency]
 * digitalplus
 * can
 * logging
 *
 * [Note]
 *
 * [Change history]
 * ver1.0 2015/ 3/ 3 The first version.
 ******************************************/

class Can;
class CanInterface{
public:
	int canSetup();
	int canWrite(int idArg,int numberArg,unsigned char dataArg[8]);
	int canSetId(int id);
	int canSetIdAll();
	virtual int canRead(int id,int number,unsigned char data[8])=0;
	virtual int canId(int id){return 0;};

	int canInterfaceSetup(Can *can);
private:
	Can *can;
};

class Can{
public:
	virtual int setup()=0;
	virtual int setupLoopBack()=0;
	int addInterface(CanInterface *interfaceArg){return addInterface(*interfaceArg);};
	virtual int addInterface(CanInterface &interfaceArg)=0;
	virtual int setId(int id)=0;
	virtual int setIdAll()=0;
	virtual int write(int id,int number,unsigned char data[8])=0;
};

#endif//CANPLUS_HPP
