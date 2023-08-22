package main

import (
	"image"
	"image/color"
	_ "image/png"
	"math"
	"time"
	"os"
	"fmt"
	"log"
	"strconv"
	"sync"

	"github.com/faiface/pixel"
	"github.com/faiface/pixel/pixelgl"
)

const (
	appleSprite int = iota
	livesSprite
	energySprite
	zombieSprite
	momoSprite
)

const (
	MON int = iota
	BON
	BUL
)

const (
	energyBlock int = -1
	G               = 20.0 // although its 9.81
	myHeightStart   = 0.5
	GROUND          = -1
	CEILING         = -3
)

const (
	emptyBlock int = iota
	wall0Block
	wall1Block
	wall2Block
	wall3Block
	wall4Block
	wall5Block
)

const (
	dDir int = iota
	centerDir
)

const (
	menuCase int = iota
	pauseCase
	gameCase
	screamCase
	editorCase
	levelChoosingCase
)

const (
	maxLevSide  = 150
	maxLevHeight = 10
	MAX = 500.0
)

var (
	fullscreen      = false
	showMap         = true
	width           = 320
	height          = 200
	scale           = 3.0
	maxLight        = 4.0
	forceLight      = 1.0
	mode            = menuCase
	energy          = 100
	energyUnit      = 0.1
	yesLight        = true
	levNow          = 0
	bonusCount      = 0
	livesStart      = 3
	die             = false
	appleEffect     = 1.0
	livesEffect     = 1.0
	mySpeed         = 3.0
	myWallDist      = 0.4
	myHeight        = myHeightStart
	myUpSpeed       = 0.0
	onGround        = true
	amountOfLevels  = 0

	as actionSquare

	pos, dir, plane Vec3
	floor map[int](*image.RGBA)
	ceiling *image.RGBA
	lives int

	monWhoKilled int

	alpha map[string](*image.RGBA)
	spriteInfo = map[int](*Sprite){}
)

type Blocks [maxLevSide][maxLevSide][maxLevHeight]int

type Level struct {
	block      Blocks
	floorsHere int
	obj        []([]Obj)
	posStart   Vec3
}
var world = [](*Level){}

type Sprite struct {
	graphs        int
	graph         int
	spriteHeight  int
	spriteWidth   int
	pic           [](*image.RGBA)
	show          bool
}

type Obj struct {
	pos      Vec3
	posStart Vec3
	dir      Vec3
	width    float64
	height   float64
	form     int
	about    int
	live     float64
	alive    bool
	taken    bool
	touched  bool
	speed    float64
}

type About struct {
	speed       float64
	through     bool
	liveMax     float64
	screamer    *image.RGBA
}
var about = []About{
	About{speed: 1.0, through: false, liveMax: 5.0},
	About{speed: 1.0, through: true, liveMax: 10.0},
}

var wallTexture = [](*image.RGBA){}



// ===================== DRAW FRAME =====================

func setLight(lightDist float64, col *color.Color) {
	d := lightDist / maxLight
	k := 1 / forceLight
	if d >= 1 {
		*col = color.RGBA{0, 0, 0, 0}
	} else {
		d = 1 - d
		root := math.Pow(d, k)
		r, g, b, a := (*col).RGBA()
		r = uint32(float64(r) / 255 * root)
		g = uint32(float64(g) / 255 * root)
		b = uint32(float64(b) / 255 * root)
		a = uint32(float64(a) / 255 * root)
		r = uint32(float64(r) / livesEffect)
		b = uint32(float64(b) / livesEffect)
		g = uint32(float64(g) / appleEffect)
		b = uint32(float64(b) / appleEffect)
		*col = color.RGBA{uint8(r), uint8(g), uint8(b), uint8(a)}
	}
}

func min123(x1 float64, x2 float64, x3 float64) int {
	if x1 < x2 {
		if x1 < x3 {
			return 1
		}
		return 3
	}
	if x2 < x3 {
		return 2
	}
	return 3
}

func nearestPlane(p float64, v float64) float64 {
	if v > 0 {
		return float64(int(p) + 1)
	}
	return float64(int(p))
}

func specRound(a float64) int {
	i := int(a)
	if a - float64(i) > 0.5 {
		return i + 1
	}
	return i
}

func signOf(a float64) float64 {
	if a > 0 {
		return 1.0
	}
	return -1.0
}

func dropInt(v Vec3) Vec3 {
	return V3(float64(int(v.X)), float64(int(v.Y)), float64(int(v.Z)))
}

// Find the crossed cube of the world
func crossBlock(finalPoint Vec3, plain int, v Vec3) (int, bool) {
	ret := dropInt(finalPoint)
	switch plain {
		case 1:
			ret.X = finalPoint.X
			if (v.X < 0) { ret.X -= 1 }
		case 2:
			ret.Y = finalPoint.Y
			if (v.Y < 0) { ret.Y -= 1 }
		case 3:
			ret.Z = finalPoint.Z
			if (v.Z < 0) { ret.Z -= 1 }
	}
	if finalPoint.X < 0.001 || finalPoint.Y < 0.001 || finalPoint.X >= maxLevSide - 1 || finalPoint.Y >= maxLevSide - 1 {
		return 0, true
	}
	if finalPoint.Z < 0.001 {
		return world[levNow].block[specRound(ret.X)][specRound(ret.Y)][0], true
	}
	if (finalPoint.Z > float64(world[levNow].floorsHere) - 0.001) {
		return CEILING, true
	}
	result := world[levNow].block[specRound(ret.X)][specRound(ret.Y)][specRound(ret.Z)]
	return result, result > 0
}

func frameGame(win *pixelgl.Window) {
	w := float64(width)
	h := float64(height)
	focus := 0.5;
	distToFrame := dir.Scaled(focus);
	parts := 10
	var wg sync.WaitGroup
	wg.Add(parts)
	var mu sync.Mutex
	for part := 0; part < parts; part++ {
		go func(mu *sync.Mutex, part int) {
			defer wg.Done()
			//po := time.Now()
			im := image.NewRGBA(image.Rect(0, 0, width/parts, height))
			for x := part*width/parts; x < (part+1)*width/parts; x++ {
				frameShiftX := V3((2 * float64(x) / w - 1), 0, 0).Rotated(plane.Angle())
				for z := 0; z < height; z++ {
					frameShiftZ := V3(0, 0, (2 * float64(z) / h - 1) * (h / w))
					lookingDirection := (distToFrame.Add(frameShiftX).Add(frameShiftZ)).Unit()
					crossedSomeWall := false
					nextX := nearestPlane(pos.X, lookingDirection.X)
					nextY := nearestPlane(pos.Y, lookingDirection.Y)
					nextZ := nearestPlane(pos.Z, lookingDirection.Z)
					iter := 0
					for !crossedSomeWall {
						lenX := MAX
						lenY := MAX
						lenZ := MAX
						lenFinal := 0.0
						if (math.Abs(lookingDirection.X) > 0.0001) {
							lenX = math.Abs(pos.X - nextX) * (1 / math.Abs(lookingDirection.X))
						}
						if (math.Abs(lookingDirection.Y) > 0.0001) {
							lenY = math.Abs(pos.Y - nextY) * (1 / math.Abs(lookingDirection.Y))
						}
						if (math.Abs(lookingDirection.Z) > 0.0001) {
							lenZ = math.Abs(pos.Z - nextZ) * (1 / math.Abs(lookingDirection.Z))
						}
						plainCrossed := min123(lenX, lenY, lenZ);
						switch plainCrossed {
							case 1:
								lenFinal = lenX
								nextX += signOf(lookingDirection.X)
							case 2:
								lenFinal = lenY
								nextY += signOf(lookingDirection.Y)
							case 3:
								lenFinal = lenZ
								nextZ += signOf(lookingDirection.Z)
						}
						finalPoint := pos.Add(lookingDirection.Scaled(lenFinal))
						dropPoint := finalPoint.Sub(dropInt(finalPoint))
						crossedBlock, flagCrossed := crossBlock(finalPoint, plainCrossed, lookingDirection)
						if flagCrossed {
							crossedSomeWall = true
							var colorTo color.Color
							switch crossedBlock {
									case 0, -1:
										bounds := floor[crossedBlock].Bounds()
										colorTo = floor[crossedBlock].At(int(float64(bounds.Max.X) * dropPoint.X),
											int(float64(bounds.Max.Y) * dropPoint.Y))
									case CEILING:
										bounds := ceiling.Bounds()
										colorTo = ceiling.At(int(float64(bounds.Max.X) * dropPoint.X),
											int(float64(bounds.Max.Y) * dropPoint.Y))
									default:
									switch plainCrossed {
										case 1:
											bounds := wallTexture[crossedBlock].Bounds()
											colorTo = wallTexture[crossedBlock].At(int(float64(bounds.Max.X) * dropPoint.Y),
												int(float64(bounds.Max.Y) * dropPoint.Z))
										case 2:
											bounds := wallTexture[crossedBlock].Bounds()
											colorTo = wallTexture[crossedBlock].At(int(float64(bounds.Max.X) * dropPoint.X),
												int(float64(bounds.Max.Y) * dropPoint.Z))
										case 3:
											bounds := wallTexture[crossedBlock].Bounds()
											colorTo = wallTexture[crossedBlock].At(int(float64(bounds.Max.X) * dropPoint.X),
												int(float64(bounds.Max.Y) * dropPoint.Y))
									}
							}
							setLight(lenFinal, &colorTo)
							im.Set(x - part*width/parts, height - z - 1, colorTo)
							
							minDistObj := maxLight + 1.0
							for k, _ := range world[levNow].obj {
								for _, m := range world[levNow].obj[k] {
									if k == MON && !m.alive || k == BON && (m.taken || !spriteInfo[m.form].show) || k == BUL && m.touched {
										continue
									}
									monDist := m.pos.Sub(pos).ReduceZ()
									if monDist.Len() > minDistObj {
										continue
									}
									minDistObj = monDist.Len()
									try := lookingDirection.Rotated(-monDist.Angle())
									if try.X > 0 {
										try = try.Scaled(monDist.Len() / try.X)
									} else {
										continue // when the monster is somewhere beside the back
									}
									lightDist := try.Len()
									if math.Abs(try.Y) > m.width || lightDist > lenFinal {
										continue // when "look" doenst cross the monster or cross a wall before
									}
									f := m.form
									g := spriteInfo[f].graph
									dZ := pos.Z + try.Z
									if dZ < m.pos.Z - m.height || dZ > m.pos.Z + m.height {
										continue
									}
									colorTo := spriteInfo[f].pic[g].At(int(float64(spriteInfo[f].spriteWidth)*(try.Y/m.width/2 + 0.5)),
										int(float64(spriteInfo[f].spriteHeight)*(-(dZ-m.pos.Z)/m.height/2 + 0.5)))
									if colorTo != spriteInfo[f].pic[g].At(0, 0) {
										setLight(lightDist, &colorTo)
										im.Set(x - part*width/parts, height - z - 1, colorTo)
									}
								}
							}
						}
						if iter > 100 {
							log.Println("iter")
							break
						}
					}
				}
			}
			var mc pixel.Vec
			p := pixel.PictureDataFromImage(im)
			c := win.Bounds().Center()
			mc = pixel.V(-w/2 + (float64(part)+0.5)*float64(width/parts), 0)
			mu.Lock()
			pixel.NewSprite(p, p.Bounds()).Draw(win, pixel.IM.Moved(c.Add(mc)).Scaled(c, scale))
			mu.Unlock()
			//dt := time.Since(po).Seconds()
			//log.Println(part, dt)
		}(&mu, part)
	}
	wg.Wait()
	im := image.NewRGBA(image.Rect(0, 0, width, height))
	coefW := w / 320
	coefH := h / 200
	drawPic(im, int(2 * coefW), int(2 * coefH), spriteInfo[appleSprite].pic[0], 0.5 * coefW)
	writeText(im, strconv.Itoa(bonusCount), int(30 * coefW), int(2 * coefH), dDir, color.RGBA{255, 0, 0, 255}, int(3 * coefW))
	drawPic(im, int(70 * coefW), int(2 * coefH), spriteInfo[livesSprite].pic[0], 0.3 * coefW)
	writeText(im, strconv.Itoa(lives), int(98 * coefW), int(2 * coefH), dDir, color.RGBA{255, 0, 0, 255}, int(3 * coefW))
	drawPic(im, int(138 * coefW), int(2 * coefH), spriteInfo[energySprite].pic[0], 0.5 * coefW)
	writeText(im, strconv.Itoa(energy)+"%", int(166 * coefW), int(2 * coefH), dDir, color.RGBA{255, 0, 0, 255}, int(3 * coefW))
	
	p := pixel.PictureDataFromImage(im)
	c := win.Bounds().Center()
	pixel.NewSprite(p, p.Bounds()).Draw(win, pixel.IM.Moved(c).Scaled(c, scale))

	if showMap {
		m := pixel.PictureDataFromImage(minimap())
		
		mc := m.Bounds().Min.Add(pixel.V(-m.Rect.W(), m.Rect.H()))
		
		pixel.NewSprite(m, m.Bounds()).
			Draw(win, pixel.IM.Moved(mc).Rotated(mc, -1.6683362599999894).
				ScaledXY(pixel.ZV, pixel.V(-scale*2*(float64(width)/320), scale*2*(float64(height)/200))))
	}
	
	win.Update()
}




// ===================== MINIMAP =====================

func getColor(x, y int) color.RGBA {
	switch world[levNow].block[x][y][0] {
	case emptyBlock:
		return color.RGBA{43, 30, 24, 255}
	case wall0Block:
		return color.RGBA{100, 89, 73, 255}
	case wall1Block:
		return color.RGBA{110, 23, 0, 255}
	case wall2Block:
		return color.RGBA{45, 103, 171, 255}
	case wall3Block:
		return color.RGBA{123, 84, 33, 255}
	case wall4Block:
		return color.RGBA{158, 148, 130, 255}
	case wall5Block:
		return color.RGBA{203, 161, 47, 255}
	case 7:
		return color.RGBA{255, 107, 0, 255}
	case 9:
		return color.RGBA{0, 0, 0, 0}
	default:
		return color.RGBA{255, 194, 32, 255}
	}
}

func minimap() *image.RGBA {
	m := image.NewRGBA(image.Rect(0, 0, 24, 26))

	//centerX := int(pos.X)
	//centerY := int(pos.Y)

	for x, row := range world[levNow].block {
		for y, _ := range row {
			c := getColor(x, y)
			if c.A == 255 {
				c.A = 96
			}
			//if (24 + x - centerX < 0 || 24 + x - centerX >= 24) && (26 + y - centerY < 0 || 26 + y - centerY >= 26) { continue }
			m.Set(x, y, c)
		}
	}

	m.Set(int(pos.X), int(pos.Y), color.RGBA{255, 0, 0, 255})
	for _, i := range world[levNow].obj[BON] {
		if !i.taken {
			switch i.form {
				case appleSprite:
					m.Set(int(i.pos.X), int(i.pos.Y), color.RGBA{0, 255, 0, 255})
				case livesSprite:
					m.Set(int(i.pos.X), int(i.pos.Y), color.RGBA{0, 255, 255, 255})
				case energySprite:
					m.Set(int(i.pos.X), int(i.pos.Y), color.RGBA{255, 255, 0, 255})
			}
		}
	}
	for _, i := range world[levNow].obj[MON] {
		if i.alive {
			m.Set(int(i.pos.X), int(i.pos.Y), color.RGBA{0, 0, 255, 255})
		}
	}


	if as.active {
		m.Set(as.X, as.Y, color.RGBA{255, 255, 255, 255})
	} else {
		m.Set(as.X, as.Y, color.RGBA{64, 64, 64, 255})
	}

	return m
}



// ===================== ACTION SQUARE =====================

func getActionSquare() actionSquare {
	pt := image.Pt(int(pos.X)+1, int(pos.Y))

	a := dir.Angle()

	switch {
	case a > 2.8 || a < -2.8:
		pt = image.Pt(int(pos.X)-1, int(pos.Y))
	case a > -2.8 && a < -2.2:
		pt = image.Pt(int(pos.X)-1, int(pos.Y)-1)
	case a > -2.2 && a < -1.4:
		pt = image.Pt(int(pos.X), int(pos.Y)-1)
	case a > -1.4 && a < -0.7:
		pt = image.Pt(int(pos.X)+1, int(pos.Y)-1)
	case a > 0.4 && a < 1.0:
		pt = image.Pt(int(pos.X)+1, int(pos.Y)+1)
	case a > 1.0 && a < 1.7:
		pt = image.Pt(int(pos.X), int(pos.Y)+1)
	case a > 1.7:
		pt = image.Pt(int(pos.X)-1, int(pos.Y)+1)
	}

	block := -1
	active := pt.X > 0 && pt.X < 23 && pt.Y > 0 && pt.Y < 23

	if active {
		block = world[levNow].block[pt.X][pt.Y][0]
	}

	return actionSquare{
		X:      pt.X,
		Y:      pt.Y,
		active: active,
		block:  block,
	}
}

type actionSquare struct {
	X      int
	Y      int
	block  int
	active bool
}

func (as actionSquare) toggle(n int) {
	if as.active {
		if world[levNow].block[as.X][as.Y][0] == 0 {
			world[levNow].block[as.X][as.Y][0] = n
		} else {
			world[levNow].block[as.X][as.Y][0] = 0
		}
	}
}

func (as actionSquare) set(n int) {
	if as.active {
		world[levNow].block[as.X][as.Y][0] = n
	}
}



// ===================== MOVE =====================

func trunc(f float64) float64 {
	return f - float64(int(f))
}

func monsterMove(dt float64) {
	for i, m := range world[levNow].obj[MON] {
		if !m.alive {
			continue
		}
		posXY := V3(pos.X, pos.Y, m.pos.Z)
		monDir := posXY.Sub(m.pos)
		if monDir.Len() < 0.5 {
			monWhoKilled = m.about
			die = true
			lives -= 1
			break
		}
		if forceLight > 1.1 && monDir.Len() <= maxLight && m.live > 0 {
			world[levNow].obj[MON][i].live -= 0.01 * monDir.Len() * forceLight
		}
		if m.live <= 0 {
			world[levNow].obj[MON][i].alive = false
			break
		}
		speed := about[m.about].speed * dt
		monDir = monDir.SetLen(speed)
		if about[m.about].through {
			world[levNow].obj[MON][i].pos = m.pos.Add(monDir)
			continue
		}
		
		signX := -1.0
		signY := -1.0
		if monDir.X > 0 { signX = 1.0 }
		if monDir.Y > 0 { signY = 1.0 }
		
		w1 := world[levNow].block[int(m.pos.X) - 1][int(m.pos.Y) - 1][0]
		w2 := world[levNow].block[int(m.pos.X)][int(m.pos.Y) - 1][0]
		w3 := world[levNow].block[int(m.pos.X) + 1][int(m.pos.Y) - 1][0]
		w4 := world[levNow].block[int(m.pos.X) - 1][int(m.pos.Y)][0]
		w6 := world[levNow].block[int(m.pos.X) + 1][int(m.pos.Y)][0]
		w7 := world[levNow].block[int(m.pos.X) - 1][int(m.pos.Y) + 1][0]
		w8 := world[levNow].block[int(m.pos.X)][int(m.pos.Y) + 1][0]
		w9 := world[levNow].block[int(m.pos.X) + 1][int(m.pos.Y) + 1][0]
		
		var vY, wY, wXY, wX, vX int
		if monDir.X >= 0 && monDir.Y >= 0 {
			vY = w7
			wY = w8
			wXY = w9
			wX = w6
			vX = w3
		} else if monDir.X >= 0 && monDir.Y < 0 {
			vY = w1
			wY = w2
			wXY = w3
			wX = w6
			vX = w9
		} else if monDir.X < 0 && monDir.Y < 0 {
			vY = w3
			wY = w2
			wXY = w1
			wX = w4
			vX = w7
		} else {
			vY = w9
			wY = w8
			wXY = w7
			wX = w4
			vX = w1
		}
		
		if world[levNow].block[int(m.pos.X)][int(m.pos.Y)][0] > 0 {
			log.Println("ERRORRR!!!")
			monWhoKilled = m.about
			die = true
			lives -= 1
			break
		}
		
		stopedX := false
		seeCorner := false
		shiftX := trunc(m.pos.X + monDir.X)
		if monDir.X >= 0 {
			shiftX = 1 - shiftX
		}
		if wX > 0 && shiftX < m.width {
			monDir.X -= (m.width - shiftX) * signX
			stopedX = true
			//log.Println("Near X wall.")
		}
		shiftY := trunc(m.pos.Y + monDir.Y)
		if monDir.Y >= 0 {
			shiftY = 1 - shiftY
		}
		if wY > 0 && shiftY < m.width {
			monDir.Y -= (m.width - shiftY) * signY
			if stopedX {
				seeCorner = true
			}
			//log.Println("Near Y wall.")
		}
		if seeCorner {
			world[levNow].obj[MON][i].pos = m.pos.Add(monDir)
			//log.Println("I`m in a corner!")
			continue
		}
		
		if wXY > 0 && wX <= 0 && wY <= 0 {
			shiftXY := V3(shiftX * signX, shiftY * signY, 0)
			if shiftXY.Len() < m.width {
				//log.Println("Left or Right?")
				if monDir.Rotated(-shiftXY.Angle()).Y > 0 {
					monDir = monDir.Rotated(math.Pi/2)
					//log.Println("From left")
				} else {
					monDir = monDir.Rotated(-math.Pi/2)
					//log.Println("From right")
				}
				world[levNow].obj[MON][i].pos = m.pos.Add(monDir)
				continue
			}
		}
		
		crossedAxisY := int(m.pos.Y) != int(m.pos.Y + monDir.Y)
		if wX <= 0 && vX > 0 && !crossedAxisY {
			shift_vX := V3(shiftX * signX, (shiftY - 1) * signY, 0)
			distanceToTheCorner, wayCrossedCorner := math.Sincos(monDir.Angle() - shift_vX.Angle())
			distanceToTheCorner *= shift_vX.Len()
			if math.Abs(distanceToTheCorner) < m.width && wayCrossedCorner > 0 {
				monDir.X = 0
				world[levNow].obj[MON][i].pos = m.pos.Add(monDir)
				//log.Println("vX", distanceToTheCorner, shift_vX, m.pos.X, m.pos.Y)
				continue
			}
		}
		
		crossedAxisX := int(m.pos.X) != int(m.pos.X + monDir.X)
		if wY <= 0 && vY > 0 && !crossedAxisX {
			shift_vY := V3((shiftX - 1) * signX, shiftY * signY, 0)
			distanceToTheCorner, wayCrossedCorner := math.Sincos(monDir.Angle() - shift_vY.Angle())
			distanceToTheCorner *= shift_vY.Len()
			if math.Abs(distanceToTheCorner) < m.width && wayCrossedCorner > 0 {
				monDir.Y = 0
				//log.Println("vY", distanceToTheCorner, shift_vY, m.pos.X, m.pos.Y)
			}
		}
		
		world[levNow].obj[MON][i].pos = m.pos.Add(monDir)
	}
}

func bonusMove(dt float64) {
	for i, b := range world[levNow].obj[BON] {
		bonDir := pos.Sub(b.pos)
		if bonDir.Len() < 0.5 && !b.taken {
			switch b.form {
				case appleSprite:
					bonusCount += 1
					world[levNow].obj[BON][i].taken = true
					appleEffect += 1.0
				case livesSprite:
					lives += 1
					world[levNow].obj[BON][i].taken = true
					livesEffect += 1.0
				case energySprite:
					if !yesLight {
						yesLight = true
						maxLight = 4.0
					}
					if energy < 100 {
						energy += 1
					}
			}
		}
	}
}

func bulletMove(dt float64) {
	for i, b := range world[levNow].obj[BUL] {
		if b.touched {
			continue
		}
		world[levNow].obj[BUL][i].pos = b.pos.Add(b.dir.Scaled(b.speed * dt));
		for j, m := range world[levNow].obj[MON] {
			if b.pos.Sub(m.pos).ReduceZ().Len() < 0.5 {
				world[levNow].obj[MON][j].alive = false
				world[levNow].obj[BUL][i].touched = true;
				break
			}
		}
		if world[levNow].block[int(b.pos.X)][int(b.pos.Y)][0] != 0 {
			world[levNow].obj[BUL][i].touched = true;
		}
	}
}



// ===================== GAME RUN =====================

func run() {
	cfg := pixelgl.WindowConfig{
		Bounds:      pixel.R(0, 0, float64(width)*scale, float64(height)*scale),
		VSync:       true,
		Undecorated: false,
	}

	if fullscreen {
		cfg.Monitor = pixelgl.PrimaryMonitor()
	}

	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		panic(err)
	}

	win.SetCursorVisible(false)

	for !win.Closed() {
		switch mode {
			case gameCase:
				if gamePress(win) {
					return
				}
			case menuCase, pauseCase:
				if menuPress(win) {
					return
				}
			case levelChoosingCase:
				if levelChoosingPress(win) {
					return
				}
			case screamCase:
				if screamPress(win) {
					return
				}
		}
	}
}

func lenSpec(text string) (int) {
	ret := 0
	for _ = range text {
		ret += 1
	}
	return ret
}

func drawPic(im *image.RGBA, x int, y int, imFrom *image.RGBA, scale float64) {
	bounds := imFrom.Bounds()
	emptyCol := imFrom.At(0, 0)
	for i := 0; i < bounds.Max.X; i++ {
		for j := 0; j < bounds.Max.Y; j++ {
			colorTo := imFrom.At(i, j)
			if colorTo != emptyCol {
				im.Set(x + int(float64(i)*scale), y + int(float64(j)*scale), colorTo)
			}
		}
	}
}

func drawPicIn(im *image.RGBA, x int, y int, x2 int, y2 int, imFrom *image.RGBA) {
	bounds := imFrom.Bounds()
	emptyCol := imFrom.At(0, 0)
	w := x2 - x
	h := y2 - y
	bw := bounds.Max.X
	bh := bounds.Max.Y
	for i := 0; i < w; i++ {
		for j := 0; j < h; j++ {
			colorTo := imFrom.At(i*bw/w, j*bh/h)
			if colorTo != emptyCol {
				im.Set(x + i, y + j, colorTo)
			}
		}
	}
}

func writeText(im *image.RGBA, text string, x int, y int, dir int, col color.Color, scale int) {
	xStart := x
	yStart := y
	if dir == centerDir {
		xStart = x - lenSpec(text)*(6*scale)/2
		yStart = y - (4*scale)
	}
	ind := 0
	for _, s := range text {
		emptyCol := alpha[string(s)].At(4, 7)
		for i := 0; i < (5*scale); i++ {
			for j := 0; j < (8*scale); j++ {
				colorTo := alpha[string(s)].At(i/scale, j/scale)
				if colorTo != emptyCol {
					im.Set(xStart + ind * ((5 + 1)*scale) + i, yStart + j, col)
				}
			}
		}
		ind += 1
	}
}

func screamPress(win *pixelgl.Window) (bool) {
	startMouse := pixel.V(float64(width)/2, float64(height)/2)
	win.SetMousePosition(startMouse)
	timer := 0

	for !win.Closed() && mode == screamCase {
		win.Clear(color.Black)
		
		// add NORMAL delay!
		timer += 1
		if timer > 50 {
			if die {
				mode = menuCase
			} else {
				mode = gameCase
			}
		}

		im := image.NewRGBA(image.Rect(0, 0, width, height))
		imFrom := about[monWhoKilled].screamer
		bx := imFrom.Bounds().Max.X
		by := imFrom.Bounds().Max.Y

		for i := 0; i < width; i++ {
			for j := 0; j < height; j++ {
				im.Set(i, j, imFrom.At(i * bx / width, j * by / height))
			}
		}

		p := pixel.PictureDataFromImage(im)

		c := win.Bounds().Center()
		pixel.NewSprite(p, p.Bounds()).Draw(win, pixel.IM.Moved(c).Scaled(c, scale))

		win.Update()
	}

	return false
}

func is(a int, b int) (int) {
	if a == b {
		return 1
	}
	return 0
}

func isnt(a int, b int) (int) {
	if a != b {
		return 1
	}
	return 0
}

func frameMenu(pointer int, graph int) *image.RGBA {
	coefW := float64(width) / 320
	//coefH := float64(height) / 200
	im := image.NewRGBA(image.Rect(0, 0, width, height))
	switch mode {
		case menuCase:
			writeText(im, "ИГРАТЬ", width/2, height/3, centerDir,
				color.RGBA{50*uint8(isnt(pointer, 0)), uint8((150 + 3*graph)*is(pointer, 0)), 0, 255}, int(2 * coefW))
		case pauseCase:
			writeText(im, "ПРОДОЛЖИТЬ", width/2, height/3, centerDir,
				color.RGBA{50*uint8(isnt(pointer, 0)), uint8((150 + 3*graph)*is(pointer, 0)), 0, 255}, int(2 * coefW))
	}
	writeText(im, "ВЫБРАТЬ УРОВЕНЬ", width/2, height/2, centerDir,
		color.RGBA{50*uint8(isnt(pointer, 1)), uint8((150 + 3*graph)*is(pointer, 1)), 0, 255}, int(2 * coefW))
	writeText(im, "ВЫЙТИ ИЗ ИГРЫ", width/2, height*2/3, centerDir,
		color.RGBA{50*uint8(isnt(pointer, 2)), uint8((150 + 3*graph)*is(pointer, 2)), 0, 255}, int(2 * coefW))
	return im
}

func menuPress(win *pixelgl.Window) (bool) {
	startMouse := pixel.V(float64(width)/2, float64(height)/2)
	win.SetMousePosition(startMouse)

	pointer := 0
	graph := 0
	pressUpTime := 0
	pressDownTime := 0

	for !win.Closed() && (mode == menuCase || mode == pauseCase) {
		if win.JustPressed(pixelgl.KeyEscape) || win.JustPressed(pixelgl.KeyQ) {
			return true
		}

		win.Clear(color.Black)
		graph = (graph + 1) % 30

		if win.Pressed(pixelgl.KeyUp) || win.Pressed(pixelgl.KeyW) {
			if pressUpTime == 0 {
				pointer = (pointer + 2) % 3
			}
			pressUpTime = (pressUpTime + 1) % 20
		} else {
			pressUpTime = 0
		}

		if win.Pressed(pixelgl.KeyDown) || win.Pressed(pixelgl.KeyS) {
			if pressDownTime == 0 {
				pointer = (pointer + 1) % 3
			}
			pressDownTime = (pressDownTime + 1) % 20
		} else {
			pressDownTime = 0
		}

		if win.JustPressed(pixelgl.KeyEnter) || win.JustPressed(pixelgl.KeySpace) {
			switch pointer {
				case 0:
					if mode == menuCase {
						reset(0)
						levNow = 0
					}
					mode = gameCase
				case 1:
					mode = levelChoosingCase
				case 2:
					return true
			}
		}

		p := pixel.PictureDataFromImage(frameMenu(pointer, graph))

		c := win.Bounds().Center()
		pixel.NewSprite(p, p.Bounds()).Draw(win, pixel.IM.Moved(c).Scaled(c, scale))

		win.Update()
	}

	return false
}

func frameLevelMenu(pointer int, graph int) *image.RGBA {
	coefW := float64(width) / 320
	im := image.NewRGBA(image.Rect(0, 0, width, height))
	writeText(im, "УРОВЕНЬ 1", width/2, height/3, centerDir,
		color.RGBA{50*uint8(isnt(pointer, 0)), uint8((150 + 3*graph)*is(pointer, 0)), 0, 255}, int(2 * coefW))
	writeText(im, "УРОВЕНЬ 2", width/2, height/2, centerDir,
		color.RGBA{50*uint8(isnt(pointer, 1)), uint8((150 + 3*graph)*is(pointer, 1)), 0, 255}, int(2 * coefW))
	writeText(im, "НАЗАД", width/2, height*2/3, centerDir,
		color.RGBA{50*uint8(isnt(pointer, 2)), uint8((150 + 3*graph)*is(pointer, 2)), 0, 255}, int(2 * coefW))
	return im
}

func levelChoosingPress(win *pixelgl.Window) (bool) {
	startMouse := pixel.V(float64(width)/2, float64(height)/2)
	win.SetMousePosition(startMouse)

	pointer := 0
	graph := 0
	pressUpTime := 0
	pressDownTime := 0

	for !win.Closed() && mode == levelChoosingCase {
		if win.JustPressed(pixelgl.KeyEscape) || win.JustPressed(pixelgl.KeyQ) {
			return true
		}

		win.Clear(color.Black)
		graph = (graph + 1) % 30

		if win.Pressed(pixelgl.KeyUp) || win.Pressed(pixelgl.KeyW) {
			if pressUpTime == 0 {
				pointer = (pointer + 2) % 3
			}
			pressUpTime = (pressUpTime + 1) % 20
		} else {
			pressUpTime = 0
		}

		if win.Pressed(pixelgl.KeyDown) || win.Pressed(pixelgl.KeyS) {
			if pressDownTime == 0 {
				pointer = (pointer + 1) % 3
			}
			pressDownTime = (pressDownTime + 1) % 20
		} else {
			pressDownTime = 0
		}

		if win.JustPressed(pixelgl.KeyEnter) || win.JustPressed(pixelgl.KeySpace) {
			switch pointer {
				case 0:
					reset(pointer)
					levNow = pointer
					mode = gameCase
				case 1:
					reset(pointer)
					levNow = pointer
					mode = gameCase
				case 2:
					mode = menuCase
			}
		}

		p := pixel.PictureDataFromImage(frameLevelMenu(pointer, graph))

		c := win.Bounds().Center()
		pixel.NewSprite(p, p.Bounds()).Draw(win, pixel.IM.Moved(c).Scaled(c, scale))

		win.Update()
	}

	return false
}

func gamePress(win *pixelgl.Window) (bool) {
	last := time.Now()

	startMouse := pixel.V(float64(width)/2, float64(height)/2)
	win.SetMousePosition(startMouse)
	
	pressShootTime := 0

	for !win.Closed() && mode == gameCase {
		dt := time.Since(last).Seconds()
		//log.Println(dt)
		last = time.Now()
		if die && lives > 0 {
			mode = screamCase
			die = false
			pos = world[levNow].posStart
			yesLight = true
			energy = 100
			energyUnit = 0.1
			maxLight = 4.0
			forceLight = 1.0
		}

		if win.JustPressed(pixelgl.KeyEscape) || win.JustPressed(pixelgl.KeyP) {
			mode = pauseCase
		}

		if appleEffect > 1.01 {
			appleEffect -= 0.2
		}
		if livesEffect > 1.01 {
			livesEffect -= 0.2
		}
		
		// Jumping physics
		pos.Z += myUpSpeed * dt
		if pos.Z - myHeightStart > 0.001 &&
			world[levNow].block[int(pos.X)][int(pos.Y)][int(pos.Z - myHeightStart - 0.001)] <= 0 {
			myUpSpeed -= G * dt // in the air
			if pos.Z > float64(world[levNow].floorsHere) - 0.1 ||
				world[levNow].block[int(pos.X)][int(pos.Y)][int(pos.Z + 0.1)] > 0 {
				pos.Z -= (myUpSpeed * dt * 2)
				myUpSpeed = -myUpSpeed // touched ceiling
			}
		} else {
			pos.Z = float64(int(pos.Z)) + myHeightStart // touched ground
			myUpSpeed = 0
			onGround = true
		}

		if die && lives == 0 {
			die = false
			mode = screamCase
		}

		if yesLight {
			energyUnit -= maxLight * forceLight * 0.01
			if energyUnit < 0 {
				energyUnit = 1
				if energy > 0 {
					energy -= 1
				} else {
					yesLight = false
				}
			}
		} else {
			maxLight = 0
		}

		spriteInfo[appleSprite].graph = (spriteInfo[appleSprite].graph + 1) %
			spriteInfo[appleSprite].graphs

		win.Clear(color.Black)

		as = getActionSquare()

		monsterMove(dt)
		bonusMove(dt)
		bulletMove(dt)

		newMousePosition := win.MousePosition()
		diffMouse := newMousePosition.Sub(startMouse).X
		win.SetMousePosition(startMouse)

		if win.Pressed(pixelgl.KeyUp) || win.Pressed(pixelgl.KeyW) {
			moveForward(mySpeed * dt)
		}

		if win.Pressed(pixelgl.KeyLeft) || win.Pressed(pixelgl.KeyA) {
			moveLeft(mySpeed * dt)
		}

		if win.Pressed(pixelgl.KeyDown) || win.Pressed(pixelgl.KeyS) {
			moveBackwards(mySpeed * dt)
		}

		if win.Pressed(pixelgl.KeyRight) ||  win.Pressed(pixelgl.KeyD) {
			moveRight(mySpeed * dt)
		}

		if win.Pressed(pixelgl.KeyU) || diffMouse > 0.01 {
			turnRight(0.2 * dt * diffMouse)
		}

		if win.Pressed(pixelgl.KeyI) || diffMouse < -0.01 {
			turnLeft(-0.2 * dt * diffMouse)
		}
		
		if win.Pressed(pixelgl.KeySpace) && onGround {
			onGround = false
			myUpSpeed += 8.0
		}

		if yesLight {
			/*if win.Pressed(pixelgl.KeyKPSubtract) {
				if maxLight > 4.01 {
					maxLight -= 0.1
				}
			}
			if win.Pressed(pixelgl.KeyKPAdd) {
				if maxLight < 9.99 {
					maxLight += 0.1
				}
			}*/
			if win.Pressed(pixelgl.MouseButtonLeft) {
				if forceLight < 1.99 {
					forceLight += 0.3
				}
				if maxLight < 10.99 {
					maxLight += 0.3
				}
			} else {
				if forceLight > 1.01 {
					forceLight -= 0.1
				}
				if maxLight > 4.01 {
					maxLight -= 0.1
				}
			}
		}

		if win.JustPressed(pixelgl.KeyM) {
			showMap = !showMap
		}

		if win.JustPressed(pixelgl.KeyB) {
			as.toggle(3)
		}
		
		if win.Pressed(pixelgl.MouseButtonRight) {
			if pressShootTime == 0 && bonusCount > 0 {
				bonusCount -= 1
				shoot();
			}
			pressShootTime = (pressShootTime + 1) % 5
		} else {
			pressShootTime = 0
		}
		
		po := time.Now()
		frameGame(win)
		dd := time.Since(po).Seconds()
		log.Println("Full render time: ", dd)
	}
	return false
}



// ===================== BUTTON ACTIONS =====================

func shoot() {
	world[levNow].obj[BUL] = append(world[levNow].obj[BUL], Obj{pos: pos, dir: dir, touched: false, width: 0.1, height: 0.1, speed: 6.0})
}

func moveForward(s float64) {
	if world[levNow].block[int(pos.X+dir.X*(s+myWallDist))][int(pos.Y)][int(pos.Z - myHeightStart)] <= 0 {
		pos.X += dir.X * s
	}
	if world[levNow].block[int(pos.X)][int(pos.Y+dir.Y*(s+myWallDist))][int(pos.Z - myHeightStart)] <= 0 {
		pos.Y += dir.Y * s
	}
}

func moveLeft(s float64) {
	if world[levNow].block[int(pos.X-plane.X*(s+myWallDist))][int(pos.Y)][int(pos.Z - myHeightStart)] <= 0 {
		pos.X -= plane.X * s
	}
	if world[levNow].block[int(pos.X)][int(pos.Y-plane.Y*(s+myWallDist))][int(pos.Z - myHeightStart)] <= 0 {
		pos.Y -= plane.Y * s
	}
}

func moveBackwards(s float64) {
	if world[levNow].block[int(pos.X-dir.X*(s+myWallDist))][int(pos.Y)][int(pos.Z - myHeightStart)] <= 0 {
		pos.X -= dir.X * s
	}
	if world[levNow].block[int(pos.X)][int(pos.Y-dir.Y*(s+myWallDist))][int(pos.Z - myHeightStart)] <= 0 {
		pos.Y -= dir.Y * s
	}
}

func moveRight(s float64) {
	if world[levNow].block[int(pos.X+plane.X*(s+myWallDist))][int(pos.Y)][int(pos.Z - myHeightStart)] <= 0 {
		pos.X += plane.X * s
	}
	if world[levNow].block[int(pos.X)][int(pos.Y+plane.Y*(s+myWallDist))][int(pos.Z - myHeightStart)] <= 0 {
		pos.Y += plane.Y * s
	}
}

func turnRight(s float64) {
	oldDirX := dir.X
	dir.X = dir.X*math.Cos(-s) - dir.Y*math.Sin(-s)
	dir.Y = oldDirX*math.Sin(-s) + dir.Y*math.Cos(-s)
	plane = dir.Rotated(-math.Pi / 2);
}

func turnLeft(s float64) {
	oldDirX := dir.X
	dir.X = dir.X*math.Cos(s) - dir.Y*math.Sin(s)
	dir.Y = oldDirX*math.Sin(s) + dir.Y*math.Cos(s)
	plane = dir.Rotated(-math.Pi / 2);
}



// ===================== READ INFO =====================

func getPic(path string) (bool, *image.RGBA) {
	file, err := os.Open(path)
	if err != nil {
		return true, image.NewRGBA(image.Rect(0, 0, 1, 1))
	}
	defer file.Close()
	pic, _, err := image.Decode(file)
	if err != nil {
		fmt.Println(err.Error())
		return true, image.NewRGBA(image.Rect(0, 0, 1, 1))
	}
	bounds := pic.Bounds()
	to := image.NewRGBA(image.Rect(0, 0, bounds.Max.X, bounds.Max.Y))
	for i := 0; i < bounds.Max.X; i++ {
		for j := 0; j < bounds.Max.Y; j++ {
			to.Set(i, j, pic.At(i, j))
		}
	}
	return false, to
}

func getBonus(form int, spritesAmount int, path string, show bool) int {
	spriteInfo[form] = &Sprite{}
	picErr, a := getPic(path)
	if picErr {
		return 1
	}
	bounds := a.Bounds()
	bh := bounds.Max.Y
	bw := bounds.Max.X / spritesAmount
	spriteInfo[form].spriteHeight = bh
	spriteInfo[form].spriteWidth = bw
	spriteInfo[form].graphs = spritesAmount
	spriteInfo[form].graph = 0
	spriteInfo[form].show = show
	spriteInfo[form].pic = [](*image.RGBA){}
	for k := 0; k < spritesAmount; k++ {
		spriteInfo[form].pic = append(spriteInfo[form].pic,
			image.NewRGBA(image.Rect(0, 0, bw, bh)))
		for i := 0; i < bw; i++ {
			for j := 0; j < bh; j++ {
				spriteInfo[form].pic[k].Set(i, j, a.At(k*bw + i, j))
			}
		}
	}
	return 0
}

func getAlpha(w int, h int) (int, map[string](*image.RGBA)) {
	res := map[string](*image.RGBA){}
	picErr, a := getPic("tex/alpha.png")
	if picErr {
		return 1, nil
	}
	allStr := []string{
		"АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ",
		"ABCDEFGHIJKLMNOPQRSTUVWXYZ",
		".,\"-()[]!?_ :;/\\$*#%+|=",
		"0123456789",
	}
	for iInd, iGird := range allStr {
		jInd := 0
		for _, jGird := range iGird {
			res[string(jGird)] = image.NewRGBA(image.Rect(0, 0, w, h))
			for i := 0; i < w; i++ {
				for j := 0; j < h; j++ {
					res[string(jGird)].Set(i, j, a.At(jInd * w + i, iInd * h + j))
				}
			}
			jInd += 1
		}
	}
	return 0, res
}

func getLevel(path string) (*Level) {
	lev := Level{}
	lev.obj = append(lev.obj, []Obj{}, []Obj{}, []Obj{})
	for fl := 0; true; fl++ {
		picErr, levPic := getPic(path + "-" + strconv.Itoa(fl) + ".png")
		//log.Println(path + "-" + strconv.Itoa(fl) + ".png")
		if picErr {
			lev.floorsHere = fl
			break
		}
		bounds := levPic.Bounds()
		for i := 0; i < bounds.Max.X; i++ {
			for k := 2; k < bounds.Max.Y; k++ {
				j := bounds.Max.Y + 1 - k
				colorTo := levPic.At(i, k)
				lev.block[i][j-2][fl] = 0
				switch colorTo {
					case levPic.At(1, 0):
						lev.block[i][j-2][fl] = wall0Block
					case levPic.At(2, 0):
						lev.posStart = V3(float64(i)+0.5, float64(j)+0.5-2, myHeight)
					case levPic.At(3, 0):
						lev.obj[BON] = append(lev.obj[BON], Obj{pos: V3(float64(i)+0.5, float64(j)+0.5-2, float64(fl)+0.5),
							taken: false, form: energySprite})
						lev.block[i][j-2][fl] = energyBlock
					case levPic.At(4, 0):
						lev.obj[BON] = append(lev.obj[BON], Obj{pos: V3(float64(i)+0.5, float64(j)+0.5-2, float64(fl)+0.5),
							taken: false, form: livesSprite, width: 0.2, height: 0.2})
					case levPic.At(5, 0):
						lev.obj[BON] = append(lev.obj[BON], Obj{pos: V3(float64(i)+0.5, float64(j)+0.5-2, float64(fl)+0.5),
							taken: false, form: appleSprite, width: 0.2, height: 0.2, speed: 0.3})
					case levPic.At(6, 0):
						lev.block[i][j-2][fl] = wall2Block
					case levPic.At(7, 0):
						lev.block[i][j-2][fl] = wall3Block
					case levPic.At(8, 0):
						lev.block[i][j-2][fl] = wall4Block
					case levPic.At(9, 0):
						lev.block[i][j-2][fl] = wall5Block
					case levPic.At(0, 1):
						lev.obj[MON] = append(lev.obj[MON], Obj{posStart: V3(float64(i)+0.5, float64(j)+0.5-2, float64(fl)+0.4),
							width: 0.4, height: 0.4, form: zombieSprite, about: 0})
					case levPic.At(1, 1):
						lev.obj[MON] = append(lev.obj[MON], Obj{posStart: V3(float64(i)+0.5, float64(j)+0.5-2, float64(fl)+0.4),
							width: 0.4, height: 0.4, form: momoSprite, about: 1})
				}
			}
		}
	}
	return &lev
}

func reset(lev int) {
	pos = world[lev].posStart
	dir = V3(-1.0, 0.0, 0.0)
	plane = V3(0.0, 1.0, 0.0)
	yesLight = true
	energy = 100
	energyUnit = 0.1
	maxLight = 4.0
	forceLight = 1.0
	for i, m := range world[lev].obj[MON] {
		world[lev].obj[MON][i].pos = m.posStart
		world[lev].obj[MON][i].live = about[m.about].liveMax
		world[lev].obj[MON][i].alive = true
	}
	bonusCount = 0
	for i := range world[lev].obj[BON] {
		world[lev].obj[BON][i].taken = false
	}
	lives = livesStart
}

func main() {
	for true {
		lev := getLevel("lev/level0" + strconv.Itoa(amountOfLevels + 1))
		if lev.floorsHere == 0 {
			if amountOfLevels == 0 {
				log.Println("Error: no level files.")
				return
			}
			break
		}
		world = append(world, lev)
		amountOfLevels++
	}

	picErr := true
	pic := image.NewRGBA(image.Rect(0, 0, 1, 1))

	for _, m := range world[0].obj[MON] { // correct 0
		picErr, pic = getPic("tex/screamer" + strconv.Itoa(m.about) + ".png")
		if picErr {
			return
		}
		about[m.about].screamer = pic
	}

	for i := 0; true; i++ {
		picErr, pic = getPic("tex/wall" + strconv.Itoa(i) + ".png")
		if picErr {
			break
		}
		wallTexture = append(wallTexture, pic)
	}

	floor = map[int](*image.RGBA){}
	picErr, pic = getPic("tex/floor0.png")
	if picErr {
		log.Println("Error: no floor files.")
		return
	}
	floor[0] = pic
	picErr, pic = getPic("tex/floor1.png")
	if picErr {
		log.Println("Error: no floor files.")
		return
	}
	floor[-1] = pic
	picErr, pic = getPic("tex/ceiling.png")
	if picErr {
		log.Println("Error: no ceiling file.")
		return
	}
	ceiling = pic
	
	bonusErr := 0
	bonusErr, alpha = getAlpha(5, 8)
	bonusErr += getBonus(appleSprite, 34, "tex/bonus.png", true)
	bonusErr += getBonus(energySprite, 1, "tex/energy.png", false)
	bonusErr += getBonus(livesSprite, 1, "tex/lives.png", true)
	bonusErr += getBonus(zombieSprite, 1, "tex/mon0.png", true)
	bonusErr += getBonus(momoSprite, 1, "tex/mon1.png", true)
	if bonusErr > 0 {
		log.Println("Error: no bonus files.")
		return
	}

	pixelgl.Run(run)
}
