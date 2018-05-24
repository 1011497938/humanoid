#ifndef FIELDGEOMETRY_HPP
#define FIELDGEOMETRY_HPP

const static int m_fieldLength = 900;
const static int m_fieldWidth = 600;
const static int m_goalDepth = 60;
const static int m_goalWidth = 260;
const static int m_goalHeight = 180;
const static int m_goalAreaLength = 100;
const static int m_goalAreaWidth = 500;
const static int m_penaltyMarkDistance = 210;
int m_centerCircleDiameter = 150;
int m_borderStripWidth = 70;
int m_lineWidth = 5;

int m_imageWidth = m_fieldLength + m_borderStripWidth * 2;
int m_imageHeight = m_fieldWidth + m_borderStripWidth * 2;

#endif // FIELDGEOMETRY_HPP
