/*
 * Protocol.h
 *
 *  Created on: 18 июл. 2019 г.
 *      Author: Игорь
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>

#define PROTOCOL_VERSION					1	//текущая версия протокола
#define PROTOCOL_PREFIX_SIZE				5	//длина префикса [CMD]
#define PROTOCOL_HEADER_LEN					8	//длина заголовка комманды

//основные комманды
#define PROTOCOL_CMD_PROTOCOL_VERSION		0	//запрос версии протокола
#define PROTOCOL_CMD_PRESET_WRITE			1	//запись текущего пресета в озу
#define PROTOCOL_CMD_PRESET_READ_EEPROM		2	//чтение текущего пресета из еепром
#define PROTOCOL_CMD_PRESET_WRITE_EEPROM	3	//запись в еепром текущего пресета

#define PROTOCOL_CMD_READ_SYS				4	//чтение системных настроек из еепром
#define PROTOCOL_CMD_WRITE_SYS				5	//запись системных настроек в  еепром

#define PROTOCOL_CMD_RESET_PRESET			6	//сброс пресета к дефолтному значению

//комманды плеера

#define PROTOCOL_CMD_PLAYER_END_DIR			 7	//конец папки-для быстрого индексирования
#define PROTOCOL_CMD_PLAYER_USB_SERIAL		 8	//серийник USB диска для целей индексирования
#define PROTOCOL_CMD_PLAYER_USB_GET_FREE	 9	//свободное место на диске для целей индексирования

#define PROTOCOL_CMD_PLAYER_PREV			10	//плеер предидущий файл
#define PROTOCOL_CMD_PLAYER_NEXT			11	//плеер следующий файл
#define PROTOCOL_CMD_PLAYER_PLAY			12	//плеер играть файл
#define PROTOCOL_CMD_PLAYER_PLAY_PAUSE		13	//плеер играть или пауза
#define PROTOCOL_CMD_PLAYER_PAUSE			14	//плеер пауза
#define PROTOCOL_CMD_PLAYER_FAVORITE		15	//плеер сделать файл любимым
#define PROTOCOL_CMD_PLAYER_NOT_FAVORITE	16	//плеер сделать файл не любимым
#define PROTOCOL_CMD_PLAYER_SEEK			17	//плеер поиск внутри файла позиции в процентах

#define PROTOCOL_CMD_PLAYER_FIND_FILE		18	//плеер чтение списка файлов-найден файл
#define PROTOCOL_CMD_PLAYER_FIND_DIR		19	//плеер чтение списка файлов-найдена папка


#define PROTOCOL_CMD_PLAYER_NEXT_FOLDER		20	//плеер следующую папку
#define PROTOCOL_CMD_PLAYER_PREV_FOLDER		21	//плеер предидущая папка

#define PROTOCOL_CMD_PLAYER_SET_FOLDER		22	//плеер установить папку по имени//получить имя текущей папки
#define PROTOCOL_CMD_PLAYER_SET_FILE		23	//плеер установить файл по имени
#define PROTOCOL_CMD_PLAYER_UP_FOLDER		24	//плеер папка вверх

#define PROTOCOL_CMD_PLAYER_SEARCH_FILE		25	//плеер поиск файла
#define PROTOCOL_CMD_PLAYER_SEARCH_FOLDER	26	//плеер поиск папки

#define PROTOCOL_CMD_PLAYER_ID3_FRAME		27	//фрейм с информацией ID3

#define PROTOCOL_CMD_PLAYER_POSITION		28	//позиция файла в секундах uint32
#define PROTOCOL_CMD_PLAYER_SET_POSITION	29	//позиция файла в процентах для перемотки со смартфона

#define PROTOCOL_CMD_PLAYER_DELETE			30	//плеер удалить файл

#define PROTOCOL_CMD_PLAYER_SCAN			31	//плеер взять список файлов

#define PROTOCOL_CMD_PLAYER_MIX_ON			32	//микс включен
#define PROTOCOL_CMD_PLAYER_MIX_OFF			33	//микс выключен
#define PROTOCOL_CMD_PLAYER_MIX_CHANGE		34	//микс изменить

#define PROTOCOL_CMD_PLAYER_REPEAT_TRACK	35	//повтор трека
#define PROTOCOL_CMD_PLAYER_REPEAT_OFF		36	//повтор выключен
#define PROTOCOL_CMD_PLAYER_REPEAT_CHANGE	37	//повтор изменить

#define PROTOCOL_CMD_PLAYER_CANT_DELETE		38	//невозможно удалить файл

#define PROTOCOL_CMD_PLAYER_REPEAT_FOLDER	39	//повтор папки

//комманды пульта
#define PROTOCOL_CMD_RUX_SET_VOLUME			40	//установка громкости
#define PROTOCOL_CMD_RUX_GET_VOLUME			41	//запрос громкости
#define PROTOCOL_CMD_RUX_INC_VOLUME			42	//прибавление громкости на 1 единицу
#define PROTOCOL_CMD_RUX_DEC_VOLUME			43	//убавление громкости на 1 единицу

#define PROTOCOL_CMD_RUX_SET_SUBWOOFER		44	//то же самое сабвуфер
#define PROTOCOL_CMD_RUX_GET_SUBWOOFER		45	//
#define PROTOCOL_CMD_RUX_INC_SUBWOOFER		46	//
#define PROTOCOL_CMD_RUX_DEC_SUBWOOFER		47	//

#define PROTOCOL_CMD_RUX_SET_PRESET			48	//то же самое пресет
#define PROTOCOL_CMD_RUX_GET_PRESET			49	//
#define PROTOCOL_CMD_RUX_ENABLE_PRESET		50	//
#define PROTOCOL_CMD_RUX_DISABLE_PRESET		51	//

#define PROTOCOL_CMD_RUX_SET_SOURCE			52	//то же самое источник
#define PROTOCOL_CMD_RUX_GET_SOURCE			53	//
#define PROTOCOL_CMD_RUX_INC_SOURCE			54	//
#define PROTOCOL_CMD_RUX_DEC_SOURCE			55	//

#define PROTOCOL_CMD_RUX_GET_PRESETS_LIST	60	//

//ИК пульт
#define PROTOCOL_CMD_IR_READ_EEPROM			70	//чтение настроек ИК из еепром
#define PROTOCOL_CMD_IR_WRITE_EEPROM		71	//запись настроек ИК в еепром
#define PROTOCOL_CMD_IR_CODE				72	//принята комманда с пульта

//руль
#define PROTOCOL_CMD_WHEEL_READ_EEPROM		80	//чтение настроек руля из еепром
#define PROTOCOL_CMD_WHEEL_WRITE_EEPROM		81	//запись настроек руля в еепром
#define PROTOCOL_CMD_WHEEL_CODE				82	//принята комманда с руля

//блютус
#define PROTOCOL_CMD_BT_PIN					90	//установка пароля блютуса, 4 символа


#define PROTOCOL_CMD_OK						100	//ответ ОК
#define PROTOCOL_CMD_ERROR					101	//ответ ошибка

#define PROTOCOL_CMD_SOFT_ON				102	//софтовое включение
#define PROTOCOL_CMD_SOFT_OFF 				103	//софтовое выключение

#define PROTOCOL_CMD_POWER_LOW				104	//низкое напряжение бортести
#define PROTOCOL_CMD_POWER_GOOD				105	//нормальное напряжение бортести

#define PROTOCOL_CMD_TEMP_HIGH_ERROR	    106	//высокая температура
#define PROTOCOL_CMD_TEMP				    107	//показания температуры в градусах (int8_t)

#define PROTOCOL_CMD_RESTART				110	//перезагрузка проца


//структура протокола связи
typedef struct
{
	uint8_t prefix[5];	//префикс [CMD]
	uint8_t cmd;		//комманда
	uint8_t sizediv4;	//размер заголовка+данных деленный на 4
	uint8_t crc;		//контрольная сумма
	uint8_t data;		//начало данных
}TProtocol;

#define PROTOCOL_CRC_OFFSET	7	//отступ значения CRC внутри протокола чтобы игнорить его при рассчете CRC


//обработка пришедших данных
void ProtocolProc();
void ProtocolStart();

void protocol_send_cmd(int cmd);
void protocol_send_data(int cmd,uint8_t* buf,int len);


#endif /* PROTOCOL_H_ */
