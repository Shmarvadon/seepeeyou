int a[8][8];
int b[8][8];

int c[8][8];

for (int i = 0; i < 8; i++) {   // Row select. (verticle)
    for (int j = 0; j < 8; j ++){   // Column select. (horizontal)
        for (int k = 0; k < 8; k++){    // Element select.
            c[i][j] += a[i][k] * b[k][j];
        }
    }
}