// aantallen
#define appels 10
#define peren 5
#define sinaasappels 3
#define druiven 45

// prijzen in euro
#define prijs_appel 2.5
#define prijs_peer 3
#define prijs_sinaasappel 2
#define prijs_druiven 0.2

#define btw 1.21 // factor

// prijs berekening
float prijs_berekening(float fruit_type, float fruit_prijs) {

    return fruit_type * fruit_prijs;  
}

// btw berekening
float btw_berekening(float prijs) {

    return prijs * btw;
}



void loop() {
    float prijs1 = prijs_berekening(sinaasappels, prijs_sinaasappel);
    float prijs1btw = btw_berekening(prijs1);

    float prijs2 = prijs_berekening(druiven, prijs_druiven);
    float prijs2btw = btw_berekening(prijs2);
    
}










